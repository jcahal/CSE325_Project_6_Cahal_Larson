#include <Adafruit_BNO055.h>
#include <FlexiTimer2.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include <Adafruit_GPS.h>
#include "DFR_Key.h"

Adafruit_GPS GPS(&Serial3);                   // define GPS object
DFR_Key keypad;
Servo myservo;                                // define servo object
Adafruit_BNO055 bno = Adafruit_BNO055(55);    // define BNO sensor object
LiquidCrystal lcd( 8, 9, 4, 5, 6, 7); // define lcd pins use these default values for OUR LCD

#define GPSECHO  false
#define lThreshold 5                          // Lidar Threshold
#define dThreshold 10                         // GPS Wall Threshold

// Global variables that change across functions

float bearing = 0;
int steeringAngle = 90;                         // servo initial angle (range is 0:180)
int carSpeed;
float heading = 0;                              // heading
boolean usingInterrupt = false;                 // Using interrupt for reading GPS chars
int carSpeedPin = 2;                            // pin for DC motor (PWM for motor driver)
long int lat = 33.420887 * 100000;              // GPS latitude in degree decimal * 100000 (CURRENT POSITION)
long int lon = -111.934089 * 100000;            // GPS latitude in degree decimal * 100000 (CURRENT POSITION)
long int latDestination = 33.421620 * 100000;   // reference destination (INITIAL DESTINATION)
long int lonDestination = -111.930118 * 100000; // reference destination (INITIAL DESTINATION)
imu::Vector<3> euler;                           // Vector of IMU
int localkey = 0;
int closeWall = 0;


///////////////////////////////////////// Boundary points  //////////////////////////////////////////
long int latPoint1 = 33.421846 * 100000;     // reference destination (Point1)
long int lonPoint1 =  -111.934683 * 100000;   // reference destination (Point1)

long int latPoint2 = 33.421846 * 100000;     // reference destination (Point2)
long int lonPoint2 =  -111.933382 * 100000;   // reference destination (Point2)

long int latPoint3 = 33.421147 * 100000;     // reference destination (Point3)
long int lonPoint3 =  -111.933772 * 100000;   // reference destination (Point3)

long int latPoint4 = 33.420825 * 100000;     // reference destination (Point4)
long int lonPoint4 =  -111.933824 * 100000;   // reference destination (Point4)

long int latPoint5 = 33.420823 * 100000;     // reference destination (Point5)
long int lonPoint5 =  -111.934168 * 100000;   // reference destination (Point5)

long int latPoint6 = 33.421151 * 100000;     // reference destination (Point6)
long int lonPoint6 =  -111.934220 * 100000;   // reference destination (Point6)
/////////////////////////////////////////////////////////////////////////////////////////////////////

long int lats[6] = {latPoint1, latPoint2, latPoint3, latPoint4, latPoint5, latPoint6};
long int lons[6] = {lonPoint1, lonPoint2, lonPoint3, lonPoint4, lonPoint5, lonPoint6};
double dists[6];



void setup() {
  myservo.attach(44);                         // servo is connected to pin 44
  lcd.begin( 16, 2 );                         // LCD type is 16x2 (col & row)
  Serial.begin(9600);                         // serial for monitoring
  if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF)) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  // byte c_data[22] = {0, 0, 0, 0, 0, 0, 209, 4, 9, 5, 9, 6, 0, 0, 255, 255, 255, 255, 232, 3, 1, 3};               // YOUR Calibration DATA
  byte c_data[22] = {0, 0, 0, 0, 0, 0, 82, 253, 156, 2, 80, 1, 0, 0, 0, 0, 2, 0, 232, 3, 184, 2}; // Old Main
  bno.setCalibData(c_data);                                                                                       // SET CALIBRATION DATA
  bno.setExtCrystalUse(true);

  // GPS Interrupts
  noInterrupts();
  TCCR4A = 0;
  TCCR4B = 0;
  TCNT4  = 336;             // every 1 second
  TCCR4B |= (1 << CS42);    // 256 prescaler
  TIMSK4 |= (1 << TOIE4);   // enable timer compare interrupt
  interrupts();

  // initialize GPS module
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // set RMC & GGA sentences
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);    // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);               // notify if antenna detected
  useInterrupt(true);                           // use interrupt for reading gps data

  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); // get initial heading 
  heading = euler.x() + 10.37; // convert to true north
  bearing = heading; //drive straight

  // Wait to start moving, display LAT, LON
  localkey = 0;
  while (localkey != 1) {    // wait for select button
    lcd.clear();
    localkey = keypad.getKey();
    lcd.print("LAT: ");
    lcd.print(lat);
    lcd.setCursor(0, 1);
    lcd.print("LON: "); 
    lcd.print(lon);
    delay(100);               // delay to make display visible
  }

  // Actuate interupts
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 59016;           // every 0.1 second
  TCCR1B |= (1 << CS12);    // 256 prescaler
  TIMSK1 |= (1 << TOIE1);   // enable timer compare interrupt
  interrupts();

} 
/////////////////////////////// end of setup(); ///////////////////////////////////

SIGNAL(TIMER0_COMPA_vect) {                   // Interrupt for reading GPS data. Don't change this...
  char c = GPS.read();
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;
#endif
}

void useInterrupt(boolean v) {                // Interrupt for reading GPS data. Don't change this...
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

void GPSRead() {
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }
  if (GPS.fix) {
    // read GPS Latitude in degree decimal
    // read GPS Longitude in degree decimal
    lat = GPS.latitudeDegrees * 100000;
    lon = GPS.longitudeDegrees * 100000;

  }
}

double CalculateDistancePerpendicular(double x1, double  y1, double  x2, double  y2, double  x3, double y3) {       // Function to calculate distance from the wall --- INPUTs:( Current x, Current y, Point i (x). Point i (y), Point j (x), Point j (y) )
  double d;
  double dX = x3 - x2;
  double dY = y3 - y2;
  double m = dY / dX; //slope
  double b = y2 - m * x2; // y-intercept
  double perpM = (-1) / m; // perpendicular slope
  double perpB = y1 - (x1 * perpM); // perpendicular y-intercept

  double poiX = (perpB - b) / (m - perpM); //Point of intersection x
  double poiY = (m * poiX) + b; //Point of intersection y

  double dX2 = poiX - x1;
  double dY2 = poiY - y1;
  d = sqrt((dX2 * dX2) + (dY2 * dY2));
  
  return d;                         // The output of this function is distance from the wall

}

void CalculateDistanceToWalls() {
  double dist;
  
  // Calculate distance from each Wall
  for(int i = 0; i < 6; i++) {
    int j = i + 1;
    
    if(j == 6) 
      j = 0;

    dist = CalculateDistancePerpendicular(lon, lat, (double)lons[i], (double)lats[i], (double)lons[j], (double)lats[j]);

    // Put distance in distances array.
    dists[i] = dist;
    
  }
}

void ReadHeading() { 
  // Read Heading from BNO055 sensor
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
}

void CalculateBearing() {
  closeWall = 0;
  // Calculate bearing based on distance to walls and seciton of boundary
  
  // Check if robot is in the top section
  if(lat > lats[2]) {
    // the robot's in the top section
    // worry about walls 1, 2, & 6
    
    // avoid wall 1
    if(dists[0] < 5) {
      bearing = 180;
      closeWall = 1;
    }
  
    // avoid wall 2
    if(dists[1] < 5) {
      bearing = 270;
      closeWall = 2;
    }

    // avoid wall 6
    if(dists[5] < 5) {
      bearing = 90;
      closeWall = 6;
    }
  
    // avoid wall 1 & 2
    if(dists[0] < 5 && dists[1] < 5) {
      bearing = 225;
      closeWall = 12;
    } 

    // avoid wall 1 & 6
    if(dists[0] < 5 && dists[5] < 5) {
      bearing = 135;
      closeWall = 16;
    }
  } else {
    // the robot's in the bottom section
    // worry about walls 3, 4, & 5

    // avoid wall 3
    if(dists[2] < 5) {
      bearing = 270;
      closeWall = 3;
    }
  
    // avoid wall 4
    if(dists[3] < 5) {
      bearing = 0;
      closeWall = 4;
    }

    // avoid wall 5
    if(dists[4] < 5) {
      bearing = 90;
      closeWall = 5;
    }
  
    // avoid wall 3 & 4
    if(dists[2] < 5 && dists[3] < 5) {
      bearing = 315;
      closeWall = 35;
    } 

    // avoid wall 4 & 5
    if(dists[3] < 5 && dists[4] < 5) {
      bearing = 45;
      closeWall = 45;
    }
  }
}

void CalculateSteering() { 
  // calculate steering angle based on heading and bearing
  float x = euler.x() + 10.37;
  int distL, distR;

  if (x > 360)
  {
    x -= 360;
  }

  // calculate the distance left and right to desired heading
  ///////////////////////////////////////////////////////////
  if(x > bearing) {

    distL = x - bearing;
    distR = (360 - x) + bearing;
    
  } else {

    distR = bearing - x;
    distL = (360 - bearing) + x;
    
  }
  ///////////////////////////////////////////////////////////

  // Set the steering angle
  ///////////////////////////////////////////////////////////
  if (distL < distR) { 
    steeringAngle = 60; // turn left all the way, it's closer...
    
    if(distL < 45)
     steeringAngle = 70;

    if(distL < 30)
     steeringAngle = 80;

    if(distL < 15)
     steeringAngle = 85;
    
  } else {
    steeringAngle = 125; // turn right, it's closer...

    if(distR < 45)
     steeringAngle = 115;

    if(distR < 30)
     steeringAngle = 105;

    if(distR < 15)
     steeringAngle = 100;
    
  }

  // x == ref angle, +/- 1
  if(x >= bearing - 2 && x <= bearing + 2) {
    steeringAngle = 93; // go straight
  }
}

void SetCarDirection() { 
 // Set direction (actuate)  
   carSpeed = 20;
  
  analogWrite(carSpeedPin, carSpeed); //change to carSpeed for production
  myservo.write(steeringAngle);                                                                        
}


ISR(TIMER1_OVF_vect) {        // This function is called every 0.1 seconds
  sei();                      // set interrupt flag ********VERY IMPORTANT******* you need to set the interrupt flag or programm will stuck here!!!
  TCNT1  = 59016;
  ReadHeading();                                                                // Read Heading
  CalculateBearing();                                                       // Calculate bearing
  CalculateSteering();                                                         // Calculate Steer angle
  SetCarDirection();                                                        // Set steer angle
}

ISR(TIMER4_OVF_vect) {      // This function is called every 1 second ....
  sei();                    // set interrupt flag ********VERY IMPORTANT******* you need to set the interrupt flag or programm will stuck here!!!
  TCNT4  = 336;             //   re-initialize timer value
  GPSRead();                //   read GPS data
  CalculateDistanceToWalls(); // find distance to walls
}


void printDiagnoticsOnLCD() {
  lcd.print("LAT");
  lcd.print(lat);
  lcd.setCursor(0, 1);
  lcd.print("LON"); 
  lcd.print(lon);
  lcd.setCursor(14, 0);
  lcd.print("CW"); 
  lcd.print(closeWall);
}

void loop() {
  lcd.clear();      // clear LCD
  // you can pring anything on the LCD to debug your program while you're in the field!
  printDiagnoticsOnLCD();
  delay(100);
}




