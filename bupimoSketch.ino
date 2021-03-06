/*
    Bupimo Sketch --  Low-level controller which runs on the Zumo 32U4 robot.
    The high-level controller sends command strings over a serial link.  The 
    format for these commands is as follows:

        desired_linear_speed:desired_angular_speed

    These quantities have units of m/sec and rads/sec.  Send the command "0:0"
    to stop the robot.

    Also, 3 seconds after receiving a command, the robot will stop.  So it
    needs to be continuously fed new commands to keep going.

    After receiving a command string, we will respond with the following 
    sequence of tab-separated quantities:

    odoX, odoY, odoTheta, linearSpeed, angularSpeed, compassHeading

    EXPERIMENTAL: The Zumo lacks easily reachable buttons.  As a work-around
    the front proximity sensor will be used as a "button" to reset
    the odometric estimate of position to (0, 0).

    ---------------------------------------------------------------------------

    Nicholi Shiell
    Andrew Vardy
*/

#include <Wire.h>
#include <Zumo32U4.h>
#include <string.h>

//#include "ContainerInt.h"
#include "Compass.h"
#include "EncoderOdometry.h"
#include "Gyro.h"

#define CALIBRATE_COMPASS 0 // 0 means no 1 means yes.
#define BAUDRATE     115200 // Baud rate used by the serial port
#define AVERAGE_OVER_N 50 // The number of samples used to calculate running average
#define CALIBRATION_SAMPLES 250  // The number samples used to find max/min compass reading
#define CONTROL_UPDATE_RATE 20 // Delay in milliseconds before control loop updates
#define COMMS_UPDATE_RATE 100 // Delay in milliseconds between sending sensor updates over serial
#define INPUT_SIZE 30
#define MAX_MILLIS_SINCE_LAST_COMMAND 3000

#define MAX_LINEAR_SPEED 0.5 // Measured in [meters / sec]
#define MAX_ANGULAR_SPEED 2. // Measured in [rads / sec]

// Proportional control constant for wheel speeds
//#define K_PROP 10.0
#define K_PROP 15.0

// Zumo constants
#define zumoBaseLength 0.085
#define zumoWheelRadius 0.019

// Object definitions
Zumo32U4ButtonC buttonC;
Zumo32U4Motors motors;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4Buzzer buzzer;

// These classes contain instansts of class from Zumo32u4.h
Compass * compass;
EncoderOdometry * odometer;
Gyro * gyro;

// Used to turn the robot on and off.
bool robotOnFlag = true;

// These values store motion commands and are read from the serial port
float linearSpeedDesired = 0.;
float angularSpeedDesired = 0.;
unsigned long commandTimeStamp = millis();

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

    // Use Serial when connected to the Up board over USB.  Use Serial1 when
    // connected to the Pi through Dennis's interface board.
    Serial.begin(BAUDRATE);
    Serial.setTimeout(10); // default value 1000.

    // Serial 0 is just for debugging with a host PC over USB
    //Serial.begin(BAUDRATE);
    //Serial.setTimeout(10);

    // Initialize the proximity counters
    proxSensors.initThreeSensors();

    // Initialize sensor classes
    compass = new Compass();
    odometer = new EncoderOdometry();
    gyro = new Gyro();

    // Calibrate gyro
    gyro->Calibrate();

    // Calibrate compass
    if(CALIBRATE_COMPASS){
        motors.setSpeeds(100, -100);
        compass->Calibrate();
        motors.setSpeeds(0, 0);
    }

    ledRed(0);
    ledGreen(1);
}

// This function splits the serial msg into its parts
void CommandParse(char* readFromSerial) {
    // Read each command pair
    char* command = strtok(readFromSerial, ":");

    linearSpeedDesired = atof(command);
    command = strtok(0, ":");
    angularSpeedDesired = atof(command);

    if (linearSpeedDesired > MAX_LINEAR_SPEED) linearSpeedDesired = MAX_LINEAR_SPEED;
    if (linearSpeedDesired < -MAX_LINEAR_SPEED) linearSpeedDesired = -MAX_LINEAR_SPEED;
    if (angularSpeedDesired > MAX_ANGULAR_SPEED) angularSpeedDesired = MAX_ANGULAR_SPEED;     
    if (angularSpeedDesired < -MAX_ANGULAR_SPEED) angularSpeedDesired = -MAX_ANGULAR_SPEED;
}

void UpdateProxSensors(){
    proxSensors.read();

    nLeft = proxSensors.countsLeftWithLeftLeds()+proxSensors.countsLeftWithRightLeds();   
    nFront = proxSensors.countsFrontWithLeftLeds()+proxSensors.countsFrontWithRightLeds();  
    nRight = proxSensors.countsRightWithLeftLeds()+proxSensors.countsRightWithRightLeds();

    // EXPERIMENTAL
    if (nFront < 8) {
        buzzer.playNote(NOTE_A(4), 100, 15);
        odometer->ResetPosition();
    }
}

void CalculateDesiredWheelSpeeds() {
    rightWheelDesired = (2.*linearSpeedDesired + angularSpeedDesired * zumoBaseLength) / (2.*zumoWheelRadius);
    leftWheelDesired = (2.*linearSpeedDesired - angularSpeedDesired * zumoBaseLength) / (2.*zumoWheelRadius);
}

void ZeroSpeeds() {
    linearSpeedDesired = 0.;
    angularSpeedDesired = 0.;
    CalculateDesiredWheelSpeeds();

    rightMotorValue = leftMotorValue = 0;
}

void ControlLoop() {
    // Time in seconds since beginning of run
    unsigned long currentTime = millis();

    // If there is data available from the serial port read it!
    if (Serial.readBytes(input, INPUT_SIZE) != 0) {
        CommandParse(input);
        CalculateDesiredWheelSpeeds();
        commandTimeStamp = currentTime;
    }

    if (currentTime - commandTimeStamp > MAX_MILLIS_SINCE_LAST_COMMAND) {
        // No recent commands received.  Stop the robot!
        ZeroSpeeds();
        motors.setSpeeds(0, 0);
    }

    // Check command time stamp. If expired reset motion commands to zero. 
    //if (runTime > 0. && float(currentTime - commandTimeStamp)/1000. > runTime)
    //  ZeroSpeeds();

    // Update all the sensors
    gyro->UpdateGyroReading();

    compass->UpdateCompassReading();

    odometer->Update();

    UpdateProxSensors();

    // This is the motor control part of the code
    // Calculate errors
    float errorRight = rightWheelDesired - odometer->GetRightWheelSpeed();
    float errorLeft = leftWheelDesired - odometer->GetLeftWheelSpeed();

    // Adjust wheel speeds based on error values

    // NICHOLI's APPROACH:
    //if (errorRight > 0.) rightMotorValue += 1;
    //else if (errorRight < 0.) rightMotorValue -= 1;
    //else {}
    // AV's APPROACH:
    rightMotorValue += (int)(K_PROP * errorRight);

    // NICHOLI's APPROACH:
    //if (errorLeft > 0.) leftMotorValue += 1;
    //else if (errorLeft < 0.) leftMotorValue -= 1;
    //else {}
    leftMotorValue += (int)(K_PROP * errorLeft);

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
        Serial.print(odometer->GetAngularSpeed());
        Serial.print("\t");
        Serial.print(compass->GetHeadingAvg());
        Serial.print("\t");
        Serial.print(nFront);
        Serial.print("\t");
        Serial.print("\n");

        // For debugging on serial port 0
        //Serial.print(linearSpeedDesired);
        //Serial.print("\t");
        //Serial.print(angularSpeedDesired);
        //Serial.print("\n");
    }
}

void loop() {
    // Detect button press which toggles between active and stopped mode.  
    // In active mode the robot follows commands sent from the host computer.
    // In stopped mode it ignores serial input.
    /*
       if (buttonC.getSingleDebouncedPress()) {
       if (robotOnFlag) {
       robotOnFlag = false;
       ledRed(1);
       ledGreen(0);
       } else { 
       robotOnFlag = true;
       ledRed(0);
       ledGreen(1);

    // Reset desired speeds to 0.  Otherwise, there will be some residual movement
    // from the last active state.
    ZeroSpeeds();
    odometer->ResetPosition();
    }
    }
     */

    if (robotOnFlag) {
        ControlLoop();
    } else {
        // If there is data coming in on the serial port, just read and ignore it:
        if (Serial.readBytes(input, INPUT_SIZE) != 0) { 
            // Pass
        }

        motors.setSpeeds(0, 0);
    }  
}
