/*
    BuPiZu Sketch --  FILL THIS IN!!!
    ------------------------------------------------------------------------------------------------

    Nicholi Shiell
*/

#include <Wire.h>
#include <Zumo32U4.h>
#include <string.h>

//#include "ContainerInt.h"
#include "Compass.h"
#include "EncoderOdometry.h"
#include "Gyro.h"

#define USE_COMPASS 0 // 0 means no 1 means yes.
#define BAUDRATE     57600 // Baud rate used by the serial port
#define AVERAGE_OVER_N 50 // The number of samples used to calculate running average
#define CALIBRATION_SAMPLES 250  // The number samples used to find max/min compass reading
#define CONTROL_UPDATE_RATE 20 // Delay in milliseconds before control loop updates
#define COMMS_UPDATE_RATE 100 // Delay in milliseconds between sending sensor updates over serial
#define INPUT_SIZE 30

#define MAX_LINEAR_SPEED 0.5 // Measured in [meters / sec]
#define MAX_ANGULAR_SPEED 2. // Measured in [rads / sec]

// Zumo constants
#define zumoBaseLength 0.085
#define zumoWheelRadius 0.019


// Object definitions
Zumo32U4ButtonC buttonC;
Zumo32U4Motors motors;
Zumo32U4ProximitySensors proxSensors;

// These classes contain instansts of class from Zumo32u4.h
Compass * compass;
EncoderOdometry * odometer;
Gyro * gyro;

// Used to turn the robot on and off.
bool robotOnFlag = false;

// These values store motion commands and are read from the serial port
float linearSpeedDesired = 0.;
float angularSpeedDesired = 0.;
float runTime = 0.;
unsigned long commandTimeStamp = 0;

// These have to do with serial comms.
char input[INPUT_SIZE];
unsigned long lastSerialCommTimeStamp = 0;

// These values are calculated from the motion commands
float rightWheelDesired = 0.;
float leftWheelDesired = 0.;

// These are uncalibrated values sent to the motors in the range +/-400. The
// resulting angular speeds of the wheels depend on the battery level.
int leftMotorValue = 0;
int rightMotorValue = 0;

// These variables store the counts registered by the IR proximity sensors.
int nLeft = 0;
int nRight = 0;
int nFront = 0;


void setup() {
  Wire.begin();
  
  Serial.setTimeout(10); // default value 1000.

  // Initialize the proximity counters
  proxSensors.initThreeSensors();
  
  // Initialize sensor classes
  compass = new Compass();
  odometer = new EncoderOdometry();
  gyro = new Gyro();

  // Calibrate gyro
  gyro->Calibrate();

  // Calibrate compass
  if(USE_COMPASS){
    motors.setSpeeds(100, -100);
    compass->Calibrate();
    motors.setSpeeds(0, 0);
  }
}

// This function splits the serial msg into its parts
void CommandParse(char* readFromSerial) {
  // Read each command pair
  char* command = strtok(readFromSerial, ":");
  linearSpeedDesired = atof(command);
  command = strtok(0, ":");
  angularSpeedDesired = atof(command);
  command = strtok(0, ":");
  runTime = atof(command);

  if(linearSpeedDesired > MAX_LINEAR_SPEED) linearSpeedDesired = MAX_LINEAR_SPEED;
  if(angularSpeedDesired > MAX_ANGULAR_SPEED) angularSpeedDesired = MAX_ANGULAR_SPEED;
     
}

void UpdatedProxSensors(){
  // put your main code here, to run repeatedly:
    proxSensors.read();
    
    nLeft = proxSensors.countsLeftWithLeftLeds()+proxSensors.countsLeftWithRightLeds();   
    nFront = proxSensors.countsFrontWithLeftLeds()+proxSensors.countsFrontWithRightLeds();  
    nRight = proxSensors.countsRightWithLeftLeds()+proxSensors.countsRightWithRightLeds();
}

void CalculateDesiredWheelSpeeds() {
  rightWheelDesired = (2.*linearSpeedDesired + angularSpeedDesired * zumoBaseLength) / (2.*zumoWheelRadius);
  leftWheelDesired = (2.*linearSpeedDesired - angularSpeedDesired * zumoBaseLength) / (2.*zumoWheelRadius);
}

void ControlLoop() {
  // Time in seconds since beginning of run
  unsigned long currentTime = millis();

  // If there is data available from the serial port read it!
  if (Serial.readBytes(input, INPUT_SIZE) != 0) {
    Serial.println("Command Recieved!");
    Serial.println(input);
    CommandParse(input);
    CalculateDesiredWheelSpeeds();
    commandTimeStamp = currentTime;
  }

  // Check command time stamp. If expired reset motion commands to zero. 
  if ( float(currentTime - commandTimeStamp)/1000. > runTime && runTime > 0.) {
    linearSpeedDesired = 0.;
    angularSpeedDesired = 0.;
    CalculateDesiredWheelSpeeds();
    
    rightMotorValue = leftMotorValue = 0;
  }

  // Update all the sensors
  gyro->UpdateGyroReading();
  
  compass->UpdateCompassReading();
  
  odometer->Update();

  UpdatedProxSensors();

  // This is the motor control part of the code
  // Calculate errors
  float errorRight = rightWheelDesired - odometer->GetRightWheelSpeed();
  float errorLeft = leftWheelDesired - odometer->GetLeftWheelSpeed();

  // Adjust wheel speeds based on error values
  if (errorRight > 0.) rightMotorValue += 1;
  else if (errorRight < 0.) rightMotorValue -= 1;
  else {}

  if (errorLeft > 0.) leftMotorValue += 1;
  else if (errorLeft < 0.) leftMotorValue -= 1;
  else {}

  if (leftMotorValue >= 400) leftMotorValue = 400;
  if (leftMotorValue <= -400) leftMotorValue = -400;

  if (rightMotorValue >= 400) rightMotorValue = 400;
  if (rightMotorValue <= -400) rightMotorValue = -400;

  motors.setSpeeds(leftMotorValue, rightMotorValue);
  
  // Send sensor data over serial port.
  if ( (currentTime -  lastSerialCommTimeStamp) > COMMS_UPDATE_RATE) {
    lastSerialCommTimeStamp = currentTime;
   
    //Serial.print(currentTime);
    //Serial.print("\t");
    //Serial.print(linearSpeedDesired);
    //Serial.print("\t");
    Serial.print(odometer->GetX());
    Serial.print("\t");
    Serial.print(odometer->GetY());
    Serial.print("\t");
    Serial.print(odometer->GetTheta());
    Serial.print("\t");
    Serial.print(odometer->GetLinearSpeed());
    Serial.print("\t");
    //Serial.print(angularSpeedDesired);
    //Serial.print("\t");
    Serial.print(gyro->GetAngularSpeedRaw());
    Serial.print("\t");
    Serial.print(odometer->GetAngularSpeed());
    Serial.print("\t");
    Serial.print(compass->GetHeadingAvg());
    Serial.print("\t");
    Serial.print(nFront);
    Serial.print("\n");

  }
}
void loop() {

  
  if (buttonC.getSingleDebouncedPress()) {
    if (robotOnFlag) robotOnFlag = false;
    else robotOnFlag = true;
  }

  if (robotOnFlag) {
    ControlLoop();
  }
  
  else {
    //Serial.println(F("Press C button to begin..."));
    motors.setSpeeds(0, 0);
  }
  
}


