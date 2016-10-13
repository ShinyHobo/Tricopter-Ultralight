/************************************************************************************
  Filename    :   tricopterController5.ino
  Content     :   This software is for use with the Arduino Mega 2560 and the Adafruit 10DOF Breakout. The intended use of this software is
                the control and stabilization of a Y-3 configuration tricopter. Static variable, manually tuned alpha-beta kalman filters
                are used to fuse the gyroscope and accelerometer raw data to determine craft orientation. A moving average filter is used
                on each to remove high frequency vibrations caused by the engines (a mechanical vibration isolator is necessary to fully
                stabilize the readout).
  Created     :   8/19/2016
  Revised     :   10/13/2016
  Authors     :   Devon Shustarich

  Copyright (c) 2016 Devon Alexander Shustarich, Allen Mathew Huju, Tony Venditto

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

*************************************************************************************/
// ================================================================
// ===                       INITIALIZATION                     ===
// ================================================================

#include <Adafruit_Sensor.h> //Introduces unified sensor types for Adafruit sensors; can be found here: https://github.com/adafruit/Adafruit_Sensor
#include <Adafruit_LSM303_U.h> //Driver for the Adafruit LSM303 Breakout, which is a combined triple-axis accelerometer and triple-axis magnetometer; library can be found here: https://github.com/adafruit/Adafruit_LSM303DLHC
#include <Adafruit_BMP085_U.h> //Driver for the Adafruit BMP085 Breakout, which is a combined barometer and thermometer; library can be found here: https://github.com/adafruit/Adafruit_BMP085_Unified
#include <Adafruit_L3GD20_U.h> //Driver for the Adafruit L3GD20 Breakout, which is a triple-axis gyroscope, included in the Adafruit 10DOF Breakout; library can be found here: https://github.com/adafruit/Adafruit_L3GD20_U
#include <Adafruit_10DOF.h> //Driver for the Adafruit 10DOF Breakout, which includes the above sensors on a single board; library can be found here: https://github.com/adafruit/Adafruit_10DOF
#include <Encoder.h> //Encoder library for use with volumetric flow rate sensors and rotory encoder prop tachs; library can be found here: http://www.pjrc.com/teensy/td_libs_Encoder.html

/* Assign a unique ID to the sensors */
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

void initSensors() //Initializes sensors.
{
  //Boolean checks for each sensor, whether or not the sensor is currently attached
  float acclCheck = true;
  float magCheck = true;
  float barCheck = true;
  float gyroCheck = true;

  if (!accel.begin()) //accelerometer
  {
    // There was a problem detecting the LSM303 ... check your connections
    Serial.println("No accelerometer (LSM303) detected.");
    acclCheck = false;
  }
  if (!mag.begin()) //magnetometer
  {
    // There was a problem detecting the LSM303 ... check your connections
    Serial.println("No magnetometer (LSM303) detected.");
    magCheck = false;
  }
  if (!bmp.begin()) //barometer and temperature
  {
    // There was a problem detecting the BMP085 ... check your connections
    Serial.print("No barometer or temperature instrumentation (BMP085) detected.");
    barCheck = false;
  }
  if (!gyro.begin()) //gyroscope
  {
    // There was a problem detecting the L3GD20 ... check your connections
    Serial.println("No gyroscope (L3GD20) detected.");
    gyroCheck = false;
  }
  if (acclCheck && magCheck && barCheck && gyroCheck) {//If any of the vital sensors are not attached or are wired incorrectly, the program will not continue past this point.
    Serial.println("Check your wiring!");
    while (1);
  }
}

void setup(void) //
{
  Serial.begin(115200);
  initSensors(); //Attempts to initialize each sensor and will alert you to a malfunction.
}

// ================================================================
// ===                          Variables                       ===
// ================================================================

float signOfZ, gs;

float acclRoll, acclPitch;
float gyroRoll, gyroPitch;

const int storeSize = 10; //The size of the matrix each moving average filter uses to store previous position values for pitch and roll.
const int filterSize = 7; //The number of values to use for the moving average of position values for pitch and roll.

float pitchStore[storeSize] = {};
int pitchArrPos = 0;
float movAvgPitch = 0;
float pitchRawSum = 0;

float rollStore[storeSize] = {};
int rollArrPos = 0;
float movAvgRoll = 0;
float rollRawSum = 0;

float acclX, acclY, acclZ;
float gyroX, gyroY, gyroZ;
float magX, magY, magZ;

float xDrive = 0, xErr = 0, xPrevErr = 0, xInter = 0, xDeriv = 0;
float zDrive = 0, zErr = 0, zPrevErr = 0, zInter = 0, zDeriv = 0;
int xSetMin = -500, xSetMax = 500, zSetMin = -500, zSetMax = 500;
float front, rear, left, right;
float xSet = 0, ySet = 2000, zSet = 0;
float x, z;
float xZero, zZero; //Orientation zero-points for roll and pitch, respectively.
bool start = true; //Keeps track of whether or not the primary loop has been passed through one time.
float dtOld, dt; //dtOld is the recorded initial time prior to each main loop sequence; dt is the change in time since the initial recording, used in each PID controller.
float sumLeft, sumRight, sumRear;
float power = 127, ratio = 3, powerSum;

float yawAngle;

//Roll PID
float xP = .3; //Proportional constant
float xI = 0.25; //Integral constant
float xD = 0.1; //Derivative constant
float xGain = 1; //PID gain multiplier

//Pitch PID
float zP = .3; //Proportional constant
float zI = 0.25; //Integral constant
float zD = 0.1; //Derivative constant
float zGain = 1; //PID gain multiplier

const int rearMotor = 10; //The PWM pin for use with the rear motor throttle control servo.
const int leftMotor = 11; //The PWM pin for use with the left motor throttle control servo.
const int rightMotor = 12; //The PWM pin for use with the right motor throttle control servo.

Encoder volume1(3, 3); //For best performance, use pins that have interupt capability. On Arduino Mega 2560, these are pins 2, 3, 18, 19, 20, and 21.
long oldVol1Pos  = -999;

// ================================================================
// ===                        MAIN PROGRAM                      ===
// ================================================================

void initial() //Determines an orientation zero-point to correct for incorrect orientation measurements on startup.
{
  xZero = movAvgRoll; //The roll zero-point.
  zZero = movAvgPitch; //The pitch zero-point.
}

void sensorRead() {
  sensors_event_t event; //Sets up the sensor event snapshot from which data can be obtained form the gyroscope, acceleromter, and magnetometer.

  //Determines the component raw data values for each sensor.
  accel.getEvent(&event);
  acclX = event.acceleration.x;
  acclY = event.acceleration.y;
  acclZ = event.acceleration.z;
  gyro.getEvent(&event);
  gyroX = event.acceleration.x;
  gyroY = event.acceleration.y;
  gyroZ = event.acceleration.z;
  mag.getEvent(&event);
  magX = event.acceleration.x;
  magY = event.acceleration.y;
  magZ = event.acceleration.z;

  signOfZ = acclZ >= 0 ? 1.0F : -1.0F;
  gs = sqrt((acclX * acclX) + (acclY * acclY) + (acclZ * acclZ)); //Determines the total G-force acting on the craft.

  xSet = map(analogRead(A1), 0, 1023, 2000, -2000) / 100.0;//Set desired roll angle with potentiometer.
  zSet = map(analogRead(A2), 0, 1023, 2000, -2000) / 100.0;//Set desired pitch angle with potentiometer.
  
  power = map(analogRead(A4), 0, 1023, 0, 255);//Set desired total power in system from 0% to 100% with potentiometer. Will be determined by the weight of the rider.
}

int rollControl() {
  /* roll: Rotation around the lateral axis (the wing span, 'Y axis'). -180<=pitch<=180)      */
  /* roll is positive and increasing when moving upwards                                      */
  /*                                                                                          */
  /*                                 x                                                        */
  /*            roll = atan(-----------------)                                                */
  /*                          sqrt(y^2 + z^2)                                                 */
  /* where:  x, y, z are returned value from accelerometer sensor                             */

  acclRoll = (float)atan2(acclX, signOfZ * sqrt(acclY * acclY + acclZ * acclZ)) * 180 / PI; //Calculates the detected roll angle using only accelerometer data.
  gyroRoll = dt / 1000.0 * gyroZ * 180 / PI + gyroRoll; //Calculates the detected roll angle using only gyroscope data.

  if (gs > 9.4 && gs < 10.4) {
    gyroRoll = .4 * acclRoll + 0.6 * gyroRoll;//Performs static kalman filtering to combine the accelerometer and gyroscope data to obtain a more stable
                                              //roll angle. Only occurs when the Gs being experienced are close to one to prevent accelerometer coordinate
                                              //frame distortion.
  }

  rollStore[rollArrPos] = gyroRoll; //Makes the current array position equal to the latest roll measurement.

  if (rollArrPos < storeSize) { //Set the sum of the roll to the sum plus the current value minus the earliest array value. Reduces number of computations considerably.
    rollRawSum = rollRawSum - rollStore[storeSize - 1 - rollArrPos] + rollStore[rollArrPos];
  } else {
    rollRawSum = rollRawSum - rollStore[rollArrPos - filterSize] + rollStore[rollArrPos];
  }

  movAvgRoll = rollRawSum / filterSize; //Calculates the moving average pitch value.

  rollArrPos++; //Increments the array position tracker by 1.

  if (rollArrPos > storeSize - 1) { //If the array position tracker exceeds the bounds of the array, place it at the beginning of the array.
    rollArrPos = 0;
  }

  x = movAvgRoll; //x is the actual roll position of the craft.

  xErr = xSet - x; //The error that the PID controller will be effected by is the difference between xSet, the input from the user that represents the desired roll angle, and x, the current roll of the craft.

  xInter = xInter + xErr * dt; //The error integration is calculated by summing the total integration error over time with the current error  multiplied by how long the error has existed. The longer the error is there, the stronger this effects the driver force.
  if (xInter > xSetMax) { //This statement prevents the integration error from spooling up or down uncontrollably by limiting the upper and lower bounds.
    xInter = xSetMax;
  } else if (xInter < xSetMin) {
    xInter = xSetMin;
  }

  xDeriv = (x - xPrevErr) / dt; //The derivative of the error is calculated by determining the change in error by the change in time

  xDrive = xGain * (xP * xErr + xI * xInter + xD * xDeriv); //The total drive value is determined by the summation of the above PID calculations multiplied by proportional, integral, and derivative constants. 

  right = power - xDrive; //The drive value is distributed equally between the left and right engines to ensure
  left = power + xDrive;

  xPrevErr = x; //Sets the previous error value to the current roll so that the change in error over time can be roughly calculated.
}

int pitchControl() {
  /* pitch: Rotation around the longitudinal axis (the plane body, 'X axis'). -90<=roll<=90    */
  /* pitch is positive and increasing when moving downward                                     */
  /*                                                                                           */
  /*                                 y                                                         */
  /*             pitch = atan(-----------------)                                               */
  /*                          sqrt(x^2 + z^2)                                                  */
  /* where:  x, y, z are returned value from accelerometer sensor                              */

  acclPitch = (float)atan2(acclY, sqrt(acclX * acclX + acclZ * acclZ)) * 180 / PI; //Calculates the detected pitch angle using only accelerometer data.
  gyroPitch = dt / 1000.0 * gyroX * 180 / PI + gyroPitch; //Calculates the detected pitch angle using only gyroscope data data.

  if (gs > 9.4 && gs < 10.4) {
    gyroPitch = .004 * acclPitch + .996 * gyroPitch; //Performs static kalman filtering to combine the accelerometer and gyroscope data to obtain a more stable
                                                     //pitch angle. Only occurs when the Gs being experienced are close to one to prevent accelerometer coordinate
                                                     //frame distortion.
  }
  pitchStore[pitchArrPos] = gyroPitch; //Makes the current array position equal to the latest pitch measurement.

  if (pitchArrPos < storeSize) { //Set the sum of the pitch to the sum plus the current value minus the earliest array value. Reduces number of computations considerably.
    pitchRawSum = pitchRawSum - pitchStore[storeSize - 1 - pitchArrPos] + pitchStore[pitchArrPos];
  } else {
    pitchRawSum = pitchRawSum - pitchStore[pitchArrPos - filterSize] + pitchStore[pitchArrPos];
  }

  movAvgPitch = pitchRawSum / filterSize; //Calculates the moving average pitch value.

  pitchArrPos++; //Increments the array position tracker by 1.

  if (pitchArrPos > storeSize - 1) { //If the array position tracker exceeds the bounds of the array, place it at the beginning of the array.
    pitchArrPos = 0;
  }

  z = movAvgRoll; //z is the actual pitch position of the craft.
  zErr = zSet - z; //The error that the PID controller will be effected by is the difference between zSet, the input from the user that represents the desired pitch angle, and z, the current pitch of the craft.

  zInter = zInter + zErr * dt; //The error integration is calculated by summing the total integration error over time with the current error  multiplied by how long the error has existed. The longer the error is there, the stronger this effects the driver force.
  if (zInter > zSetMax) { //This statement prevents the integration error from spooling up or down uncontrollably by limiting the upper and lower bounds.
    zInter = zSetMax;
  } else if (zInter < zSetMin) {
    zInter = zSetMin;
  }

  zDeriv = (z - zPrevErr) / dt; //The derivative of the error is calculated by determining the change in error by the change in time

  zDrive = zGain * (zP * zErr + zI * zInter + zD * zDeriv); //The total drive value is determined by the summation of the above PID calculations multiplied by proportional, integral, and derivative constants. 

  rear = power - zDrive; //The drive value is distributed according to the distance between the front and center and the rear and center.
  front = power + zDrive / 2;

  zPrevErr = z; //Sets the previous error value to the current pitch so that the change in error over time can be roughly calculated.
}

void pidSum() //Sums the PID drivers from the pitch and roll functions to provide stabilization and control of the aircraft.
{
  float momArmFront; //Distance from front moment force (summation of left and right motors).
  float momArmRear; //Distance from rear moment force (rear motor).

  /*                                    Pitch free body diagram                               */
  /*                                                                                          */
  /*                     left & right                                rear                     */
  /*                          ^                                       ^                       */
  /*                          |                                       |                       */
  /*                          |  momArmFront        momArmRear        |                       */
  /*                         [M]============o========================[M]                      */
  /*                                        |                                                 */
  /*                                        |                                                 */
  /*                                        v                                                 */
  /*                                     gravity                                              */
  /*                                                                                          */

  sumRear = (power * momArmFront) / ( momArmFront + momArmRear) + rear; //The driver value to be sent to the rear motor throttle control servo.
  sumLeft = (power - sumRear) / 2 + front + left; //The driver value to be sent to the left motor throttle control servo.
  sumRight = (power - sumRear) / 2 + front + right; //The driver value to be sent to the right motor throttle control servo.

  //Need alternative limiter, based off of power of the increasing engine.
  if (sumLeft < 0) {
    sumLeft = 0;
  } else if (sumLeft > 255) {
    sumLeft = 255;
  }

  if (sumRight < 0) {
    sumRight = 0;
  } else if (sumRight > 255) {
    sumRight = 255;
  }

  if (sumRear < 0) {
    sumRear = 0;
  } else if (sumRear > 255) {
    sumRear = 255;
  }

  analogWrite(rearMotor, sumRear); //Sends the summed driver value to the rear motor throttle control servo.
  analogWrite(rightMotor, sumRight); //Sends the summed driver value to the right motor throttle control servo.
  analogWrite(leftMotor, sumLeft); //Sends the summed driver value to the left motor throttle control servo.
}

void volumeGrab() { //Determines position based on changes in states of an optical encoder. Will be used to track volume changes in the gasoline tank.
  long newVol1Pos = volume1.read() / 2; //Reads the current state of the optical encoder; divides by two to account for using only using a single direction optical encoder.
  if (newVol1Pos != oldVol1Pos) { //If the state has changed, set the position equal to the new position.
    oldVol1Pos = newVol1Pos;
  }
}

void yawControl(){
  yawAngle = map(analogRead(A3), 0, 1023, 160, 230); //Maps the potentiometer input to the pwm value sent to the yaw servo.
  analogWrite(8, yawAngle); //Set desired rear rotor angle (yaw control) with potentiometer.
}

void loop() { //The primary program loop.
  dtOld = millis(); //Records the the current time step for use in the time delta calculation later in the loop.

  sensorRead(); // obtains the current X, Y, and Z-axis state information of each the magnetometer, accelerometer, and gyroscope.

  if (start) { //Only true on the first pass through the primary loop.
    //initial(); //Determines an orientation zero-point to correct for incorrect orientation measurements.
    start = false; //Once the orientation zero-point is set, this prevents the initial() function from running again.
  } else { //Once the orientation zero-point is set, the roll and pitch functions are run to obtain the orientation information and calculate PID driver values for the motors.
    rollControl(); //determine the roll of the craft and calulate the PID driver values for the left and right.

    /*                                    Roll free body diagram                                */
    /*                                                                                          */
    /*                         left                rear               right                     */
    /*                          ^                   ^                   ^                       */
    /*                          |                   |                   |                       */
    /*                          |                   |                   |                       */
    /*                         [M]==================o==================[M]                      */
    /*                                              |                                           */
    /*                                              |                                           */
    /*                                              v                                           */
    /*                                           gravity                                        */
    /*                                                                                          */

    pitchControl(); //determine the pitch of the craft and calulate the PID driver values for the front (left and right summation) and rear.

    /*                                    Pitch free body diagram                               */
    /*                                                                                          */
    /*                     left & right                                rear                     */
    /*                          ^                                       ^                       */
    /*                          |                                       |                       */
    /*                          |                                       |                       */
    /*                         [M]============o========================[M]                      */
    /*                                        |                                                 */
    /*                                        |                                                 */
    /*                                        v                                                 */
    /*                                     gravity                                              */
    /*                                                                                          */

    pidSum(); //The driver values are passed to the pidSum() function to sum together the two PID drivers for pitch and roll, providing stabilization and control to the aircraft.
    yawControl(); //Adjusts the yaw servo and provides the yaw servo angle to adjust the rear engine power.
    volumeGrab(); //Obtains the volumetric flow rate of a single volumetric flow rate optical encoder.
  }

  dt = (millis() - dtOld) / 1000.0; //Calculates the time delta for each pass of the primary program loop by comparing the current clock time and the clock time at the beginning of the loop; dt is in seconds.
}
