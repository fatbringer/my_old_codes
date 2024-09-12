/* modified from MPU6050_ACCEL_SIMPLE
 *  
    MPU6050 Triple Axis Gyroscope & Accelerometer. Simple Accelerometer Example.
    Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/3-osiowy-zyroskop-i-akcelerometr-mpu6050.html
    GIT: https://github.com/jarzebski/Arduino-MPU6050
    Web: http://www.jarzebski.pl
    (c) 2014 by Korneliusz Jarzebski

download the github repo from the link above, go to "Sketch >> Include library >> add .zip file" then selectthat downloaded file 
*/

#include <Wire.h> // please read this https://www.arduino.cc/en/reference/wire to know the wiring 
/* 
VCC - 5v    GND - GND
SCL - A5    SDA - A4
the wire.h library makes it this way --> A4 (SDA), A5 (SCL)  */

#include <MPU6050.h> 


MPU6050 mpu;

// for using
// v = u + at AND s = ut + 1/2 at^2
// assumption is small timing differences acceleration is almost constant
// so we use the basic kinematics formulas
float initialVeloX, finalVeloX;
float accelerationX, dispX;

float initialVeloY, finalVeloY;
float accelerationY, dispY;

float accelerationZ;
float CalibAccelX, CalibAccelY, CalibAccelZ;
float gravX, gravY;

// for usage in movement_end_check
int countX = 0;
int countY = 0;

// others
int printerCount =0;
long timeMic1, timeMic2;

void setup()
{
  Serial.begin(115200);
  Serial.println("Initialize MPU6050");

  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  

//  mpu.setDLPFMode(MPU6050_DLPF_0);

  mpu.calibrateGyro();
  // If you want, you can set accelerometer offsets
  mpu.setAccelOffsetX(0);
  mpu.setAccelOffsetY(0);
  mpu.setAccelOffsetZ(1500);

  checkSettings();  
  callibrateAccelerometer();
  initialVeloX = 0;
  initialVeloY = 0;
  dispX = 0;
  dispY = 0;

} // end of setup

void checkSettings()
{
  Serial.println();

  Serial.print(" * Sleep Mode:            ");
  Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");

  Serial.print(" * Clock Source:          ");
  switch (mpu.getClockSource())
  {
    case MPU6050_CLOCK_KEEP_RESET:     Serial.println("Stops the clock and keeps the timing generator in reset"); break;
    case MPU6050_CLOCK_EXTERNAL_19MHZ: Serial.println("PLL with external 19.2MHz reference"); break;
    case MPU6050_CLOCK_EXTERNAL_32KHZ: Serial.println("PLL with external 32.768kHz reference"); break;
    case MPU6050_CLOCK_PLL_ZGYRO:      Serial.println("PLL with Z axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_YGYRO:      Serial.println("PLL with Y axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_XGYRO:      Serial.println("PLL with X axis gyroscope reference"); break;
    case MPU6050_CLOCK_INTERNAL_8MHZ:  Serial.println("Internal 8MHz oscillator"); break;
  }

  Serial.print(" * Accelerometer:         ");
  switch (mpu.getRange())
  {
    case MPU6050_RANGE_16G:            Serial.println("+/- 16 g"); break;
    case MPU6050_RANGE_8G:             Serial.println("+/- 8 g"); break;
    case MPU6050_RANGE_4G:             Serial.println("+/- 4 g"); break;
    case MPU6050_RANGE_2G:             Serial.println("+/- 2 g"); break;
  }

  Serial.print(" * Accelerometer offsets: ");
  Serial.print(mpu.getAccelOffsetX());
  Serial.print(" / ");
  Serial.print(mpu.getAccelOffsetY());
  Serial.print(" / ");
  Serial.println(mpu.getAccelOffsetZ());

} // end of checkSettings 


void loop() {
  float timediff;
//  // for checking time differences in MICROseconds
//  timeMic1 = micros();
//  Serial.print("time b/w loops: ");
//  Serial.println(timeMic1 - timeMic2);

  // resets these values to 0 at the start of each main loop so that previous values don't affect current loop
  accelerationX = 0;
  accelerationY = 0;

  int looper = 0;
  for (looper = 0; looper < 64 ; looper++ ) {
    Vector normAccel = mpu.readNormalizeAccel(); /// this takes 1 ms
    accelerationX = accelerationX + normAccel.XAxis  ;
    accelerationY = accelerationY + normAccel.YAxis  ;
  }

  accelerationX = ( accelerationX / looper ) - CalibAccelX ;
  accelerationY = ( accelerationY / looper ) - CalibAccelY ;


  acceleration_limit();
  dataDiscrimination();

  // MPU6050 data interval is 1 kHz, using T = 1/f, each call of readAcceleration takes 1ms 
  timediff = 0.00104 * looper  ;  //time per sample multiplied by number of samples (given by looper) 
  
  // calculations
  dispX = dispX + (initialVeloX * timediff) + (0.5 * accelerationX * sq(timediff)  )  ; // s = ut + 1/2 a t ^2
  dispY = dispY + (initialVeloY * timediff) + (0.5 * accelerationY * sq(timediff)  )  ; // s = ut + 1/2 a t ^2

  finalVeloX = initialVeloX + ( accelerationX * timediff ) ;    // v = u + at
  finalVeloY = initialVeloY + ( accelerationY * timediff ) ;    // v = u + at
  
  velocity_limit();
  movement_end_check();

  initialVeloX = finalVeloX;    // updates initial with final 
  initialVeloY = finalVeloY;    // updates initial with final 

  // debugging //
  printerCount++;  // each loop takes an approximate 67000 microseconds to complete, calculate wisely !
    if (printerCount == 45) {  
//  Serial.print("AccelX: ");
//  Serial.print(accelerationX);
//  Serial.print(" AccelY: ");
//  Serial.println(accelerationY);
//  Serial.print("finalVeloX: ");
//  Serial.print(finalVeloX);
//  Serial.print(" finalVeloY: ");
//  Serial.println(finalVeloY);
      Serial.print("DispX: ");
      Serial.print(dispX );
      Serial.print(" DispY: ");
      Serial.println(dispY );
      printerCount = 0;
    }


  //delay(10); // originally was 500

//  timeMic2 = micros();
//  Serial.print("Time taken for a loop: ");
//  Serial.println(timeMic2 - timeMic1);


} // end of main loop




void callibrateAccelerometer() {
  // gathers initial acceleration values, then gets average to determine how much to offset these values (like correcting zero error) 
  // assumes that the IMU is completely flat. Hence acceleration in X and Y direction should yield almost 0.
  // after this, we will do reading minus callibration.
  
  int sampleCount = 1024;
  float accelXsumma = 0;
  float accelYsumma = 0;
  float accelZsumma = 0;
  int i = 0;
  for ( i = 0; i < sampleCount ; i++ ) {
    Vector normAccel = mpu.readNormalizeAccel();
    accelXsumma = accelXsumma + normAccel.XAxis;
    accelYsumma = accelYsumma + normAccel.YAxis;
    accelZsumma = accelZsumma + normAccel.ZAxis;
  }
  CalibAccelX = accelXsumma / (sampleCount);
  CalibAccelY = accelYsumma / (sampleCount);
  Serial.print("Callibration acceleration X: ");
  Serial.print(CalibAccelX);
  Serial.print(" Callibration acceleration Y: ");
  Serial.print(CalibAccelY);
  Serial.print(" Callibration acceleration Z: ");
  Serial.println(CalibAccelZ);
} // end of calibrate Acceleormeter



void dataDiscrimination() {
  // Discrimination window applied to the X and Y  axis acceleration variable
  // you should check the standard deviation of the values first !
  // any values that fall within this specified range is treated as mechanical noise and discarded 
  float accelXStdDev = 0.100451;
  float accelYStdDev = 0.100805;
  if ((accelerationX <= accelXStdDev ) && (accelerationX >= -1 * accelXStdDev )) {
    accelerationX = 0;
    //Serial.println("accelX discriminated"); //for debugging
  }

  if ((accelerationY <= accelYStdDev ) && ( accelerationY >= -1 * accelYStdDev )) {
    accelerationY = 0;
    //Serial.println("accelY discriminated"); // for debugging 
  }
} // end of dataDiscrimination


void movement_end_check()  {
  if (accelerationX == 0) { //we count the number of acceleration samples that equals zero
    countX++;
  }
  else {
    countX = 0;
  }

  if (countX >= 8) { //if this number exceeds a certain value, we can assume that velocity is zero
    finalVeloX = 0;
    //Serial.println("finalVeloX set to 0");
  }

  if (accelerationY == 0) { //we do the same for the Y axis
    countY++;
  }
  else {
    countY = 0;
  }

  if (countY >= 8) {
    finalVeloY = 0;
    //Serial.println("finalVeloY set to 0");
  }
} // end of movement_end_check


void contrib_by_gravity() {
// calculates the contribution of gravity in the X and Y directions, then make calculations to minimise such effects.
// Relies on getting the pitch and roll for calculation. Yaw should not matter. 
// Since we are not using quarternions, gimbal lock might occur. But it also should not be relevant because if gimbal lock occurs, robot will not be functional anymore.
  Vector normGyro = mpu.readNormalizeGyro();
  gravX = 9.81 * sin(normGyro.XAxis); // gsin(theta) for object on incline  
  gravY = 9.81 * sin(normGyro.YAxis); 
  accelerationX = accelerationX - gravX;
  accelerationY = accelerationY - gravY;
} // end of contrib_by_gravity 

void velocity_limit() {
// attempt to prevent ridiculous runaway displacements with an artificial cap of expected maximum velocities
// change these values to whatever that resembles the physical limits 
float maxVeloX = 0.8;
float minVeloX = -0.8;

float maxVeloY = 0.8;
float minVeloY = -0.8;

  if (finalVeloX >= maxVeloX ) {
    finalVeloX = maxVeloX;
  }
  if (finalVeloX <= minVeloX ) {
    finalVeloX = minVeloX;
  }
  if (finalVeloY >= maxVeloY ) {
    finalVeloY = maxVeloY;
  }
  if (finalVeloY <= minVeloY) {
    finalVeloY = minVeloY ;
  }
  
} // end of velocity_limit

void acceleration_limit() {
// attempt to prevent ridiculous runaway displacements with an artificial cap of expected maximum acceleratio
// change these values to whatever that resembles the physical limits 
float maxAccelX = 3;
float minAccelX = -3;

float maxAccelY = 3;
float minAccelY = -3;

  if (accelerationX >= maxAccelX ) {
    accelerationX = maxAccelX;
  }
  if (accelerationX <= minAccelX ) {
    accelerationX= minAccelX;
  }
  if (accelerationY >= maxAccelY ) {
    accelerationY = maxAccelY;
  }
  if (accelerationY <= minAccelY) {
    accelerationY = minAccelY ;
  }
  
} // end of acceleration_limit
