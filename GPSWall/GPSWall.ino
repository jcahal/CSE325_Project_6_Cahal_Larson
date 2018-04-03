#include <Adafruit_BNO055.h>
#include <FlexiTimer2.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include <Adafruit_GPS.h>

Adafruit_GPS GPS(&Serial3);                   // define GPS object
Servo myservo;                                // define servo object
Adafruit_BNO055 bno = Adafruit_BNO055(55);    // define BNO sensor object
LiquidCrystal lcd( 8, 9, 4, 5, 6, 7); // define lcd pins use these default values for OUR LCD

#define GPSECHO  false
#define lThreshold 5                          // Lidar Threshold
#define dThreshold 10                         // GPS Wall Threshold

// Global variables that change across functions

float Bearing = 0;
int STEERANGLE = 90;                            // servo initial angle (range is 0:180)
float HEADING = 0;                              // heading
int LidarRight;                                 // LIDAR left
int LidarLeft;                                  // LIDAR right
boolean usingInterrupt = false;                 // Using interrupt for reading GPS chars
int carSpeedPin = 2;                            // pin for DC motor (PWM for motor driver)
float errorHeadingRef = 0;                      // Heading error
long int lat = 33.420887 * 100000;              // GPS latitude in degree decimal * 100000 (CURRENT POSITION)
long int lon = -111.934089 * 100000;            // GPS latitude in degree decimal * 100000 (CURRENT POSITION)
long int latDestination = 33.421620 * 100000;   // reference destination (INITIAL DESTINATION)
long int lonDestination = -111.930118 * 100000; // reference destination (INITIAL DESTINATION)

// Our Global Vars
imu::Vector<3> euler;                         // Vector of IMU


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



void setup() {
  myservo.attach(44);                         // servo is connected to pin 44
  lcd.begin( 16, 2 );                         // LCD type is 16x2 (col & row)
  Serial.begin(9600);                         // serial for monitoring
  if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF)) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  byte c_data[22] = {0, 0, 0, 0, 0, 0, 209, 4, 9, 5, 9, 6, 0, 0, 255, 255, 255, 255, 232, 3, 1, 3};               // YOUR Calibration DATA
  bno.setCalibData(c_data);                                                                                       // SET CALIBRATION DATA
  bno.setExtCrystalUse(true);

  // set timer interrupts
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 59016;           // every 0.1 second
  TCCR1B |= (1 << CS12);    // 256 prescaler
  TIMSK1 |= (1 << TOIE1);   // enable timer compare interrupt

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

}

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

ISR(TIMER4_OVF_vect) {      // This function is called every 1 second ....
  sei();                    // set interrupt flag ********VERY IMPORTANT******* you need to set the interrupt flag or programm will stuck here!!!
  TCNT4  = 336;             //   re-initialize timer value
  GPSRead();                //   read GPS data
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
    

    // Calculate distance from each Wall (call CalculateDistancePerpendicular(); )
    double dW1 = CalculateDirectionPerpendicularY(lon, lat, lonPoint1, latPoint1, lonPoint2, latPoint2);
    double dW2 = CalculateDirectionPerpendicularY(lon, lat, lonPoint2, latPoint2, lonPoint3, latPoint3);
    double dW3 = CalculateDirectionPerpendicularY(lon, lat, lonPoint3, latPoint3, lonPoint4, latPoint4);
    double dW4 = CalculateDirectionPerpendicularY(lon, lat, lonPoint4, latPoint4, lonPoint5, latPoint5);
    double dW5 = CalculateDirectionPerpendicularY(lon, lat, lonPoint5, latPoint5, lonPoint6, latPoint6);
    double dW6 = CalculateDirectionPerpendicularY(lon, lat, lonPoint6, latPoint6, lonPoint1, latPoint1);
    double perpX;
    double perpY;

    // if distance is less than the threshold
    // compute the direction vector X (call CalculateDirectionPerpendicularX(); )
    // compute the direction vector Y (call CalculateDirectionPerpendicularY(); )
    if(dW1 < 5) {
      perpX = CalculateDirectionPerpendicularX(lon, lat, lonPoint1, latPoint1, lonPoint2, latPoint2);
      perpY = CalculateDirectionPerpendicularY(lon, lat, lonPoint1, latPoint1, lonPoint2, latPoint2);
    } 
    if(dW2 < 5) {
      perpX = CalculateDirectionPerpendicularX(lon, lat, lonPoint2, latPoint2, lonPoint3, latPoint3);
      perpY = CalculateDirectionPerpendicularY(lon, lat, lonPoint2, latPoint2, lonPoint3, latPoint3);
    }
    if(dW3 < 5) {
      perpX = CalculateDirectionPerpendicularX(lon, lat, lonPoint3, latPoint3, lonPoint4, latPoint4);
      perpY = CalculateDirectionPerpendicularY(lon, lat, lonPoint3, latPoint3, lonPoint4, latPoint4);
    }
    if(dW4 < 5) {
      perpX = CalculateDirectionPerpendicularX(lon, lat, lonPoint4, latPoint4, lonPoint5, latPoint5);
      perpY = CalculateDirectionPerpendicularY(lon, lat, lonPoint4, latPoint4, lonPoint5, latPoint5);
    }
    if(dW5 < 5) {
      perpX = CalculateDirectionPerpendicularX(lon, lat, lonPoint5, latPoint5, lonPoint6, latPoint6);
      perpY = CalculateDirectionPerpendicularY(lon, lat, lonPoint5, latPoint5, lonPoint6, latPoint6);
    }
    if(dW6 < 5) {
      perpX = CalculateDirectionPerpendicularX(lon, lat, lonPoint6, latPoint6, lonPoint1, latPoint1);
      perpY = CalculateDirectionPerpendicularY(lon, lat, lonPoint6, latPoint6, lonPoint1, latPoint1);
    }
    

    // Set a new Destination according to direction vectors X & Y
    // latDestination =
    // lonDestination = 
  }
}

double CalculateDirectionPerpendicularX(double x1, double  y1, double  x2, double  y2, double  x3, double y3) {     // Function to Calculate Horizental vector ---INPUTs:( Current x, Current y, Point i (x). Point i (y), Point j (x), Point j (y) )
  double Dx;

  double dX = x3 - x2;
  double dY = y3 - y2;
  double m = dY / dX; //slope
  double b = y2 - m * x2; // y-intercept
  double perpM = (-1) / m; // perpendicular slope
  double perpB = y1 - (x1 * perpM); // perpendicular y-intercept
  double poiX = (perpB - b) / (m - perpM); //Point of intersection x
  double poiY = (m * poiX) + b; //Point of intersection y

  double x = poiX - x1;
  double y = poiY - y1;
  double magnitude = sqrt((x * x) + (y * y));
  Dx = -x \ magnitude;
  
  return Dx;                        // The output of this function is direction along x-axis
}

double CalculateDirectionPerpendicularY(double x1, double  y1, double  x2, double  y2, double  x3, double y3) {     // Function to Calculate Vertical vector   ---INPUTs:( Current x, Current y, Point i (x). Point i (y), Point j (x), Point j (y) )
  double Dy;

  double dX = x3 - x2;
  double dY = y3 - y2;
  double m = dY / dX; //slope
  double b = y2 - m * x2; // y-intercept
  double perpM = (-1) / m; // perpendicular slope
  double perpB = y1 - (x1 * perpM); // perpendicular y-intercept
  double poiX = (perpB - b) / (m - perpM); //Point of intersection x
  double poiY = (m * poiX) + b; //Point of intersection y

  double x = poiX - x1;
  double y = poiY - y1;
  double magnitude = sqrt((x * x) + (y * y));
  Dy = -y \ magnitude;
  
  return Dy ;                       // The output of this function is direction along y-axis
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

void ReadHeading() { 
  // Read Heading from BNO055 sensor
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
}

void CalculateBearing() {
  // Calculate Bearing based on current and destination coordinates
  Bearing = 90 - atan2((latDestination - lat),(lonDestination - lon)) * (180 / PI);
  if(Bearing < 0) {
    Bearing += 360;
  }
}

void CalculateSteering() { 
  // calculate steering angle based on heading and bearing
  float x = euler.x() + 10.37;

  if (x > 360)
  {
    x -= 360;
  }

  // calculate the distance left and right to desired heading
  ///////////////////////////////////////////////////////////
  if(x > Bearing) {

    distL = x - Bearing;
    distR = (360 - x) + Bearing;
    
  } else {

    distR = Bearing - x;
    distL = (360 - Bearing) + x;
    
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
  //if(x >= ref - 2 && x <= ref + 2) {
  if(x == Bearing) {
    steeringAngle = 93; // go straight
  }
}

void SetCarDirection() { 
 // Set direction (actuate)  
   carSpeed = 20;
  if(distance < 4) {
    carSpeed = 0;
  }

  
  analogWrite(carSpeedPin, carSpeed); //change to carSpeed for production
  myservo.write(steeringAngle);                                                                        
}





ISR(TIMER1_OVF_vect) {        // This function is called every 0.1 seconds
  sei();                      // set interrupt flag ********VERY IMPORTANT******* you need to set the interrupt flag or programm will stuck here!!!
  TCNT1  = 59016;
  ReadHeading();                                                                // Read Heading
  CalculateBearing();                                                       // Calculate Bearing
  CalculateSteering();                                                         // Calculate Steer angle
  SetCarDirection();                                                        // Set steer angle
}


void printHeadingOnLCD() {

}

void printLocationOnLCD() {

}



void printObstacleOnLCD() {

}

void loop() {
  lcd.clear();      // clear LCD
  // you can pring anything on the LCD to debug your program while you're in the field!
  printHeadingOnLCD();
  printLocationOnLCD();
  //  printObstacleOnLCD();
  delay(100);
}




