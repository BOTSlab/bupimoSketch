#include "ContainerInt.h"
#include <Zumo32U4.h>
#include <Wire.h>

// Gryo sensor constants
#define gyroSensitivity 0.00875


class Gyro{
 public:

  Gyro(int runningAverageSampleSize = 50);
  void Calibrate(int nCalibrationSamples = 250);
  void UpdateGyroReading(); 

  float GetAngularSpeedAvg();
  float GetAngularSpeedRaw();

 private:
  ContainerInt* xContainer;
  ContainerInt* yContainer;
  ContainerInt* zContainer;

  int xOffset;
  int yOffset;
  int zOffset;

  int xRaw, yRaw, zRaw;
  
  L3G gyro;
  
};


Gyro::Gyro(int runningAverageSampleSize){
  // Initialize hardware
  gyro.init();
  gyro.enableDefault();
  
  //Create containers
  xContainer = new ContainerInt(runningAverageSampleSize);
  yContainer = new ContainerInt(runningAverageSampleSize);
  zContainer = new ContainerInt(runningAverageSampleSize);

  xOffset = 0;
  yOffset = 0;
  zOffset = 0;

  // Fill the running average containers. The offset has not been 
  // removed yet, so the readings are garbage. I think this is preferable
  // to having a bunch of garbage in there.
  for(int i = 0; i <runningAverageSampleSize; i++){
        UpdateGyroReading();      
  }
};

void Gyro::Calibrate(int nCalibrationSamples){
   for(int index = 0; index < nCalibrationSamples; index ++){
    // Take a reading from the gyro.
    gyro.read();

    xOffset += gyro.g.x;
    yOffset += gyro.g.y;
    zOffset += gyro.g.z;
    
    delay(10);
   }

   xOffset /= nCalibrationSamples;
   yOffset /= nCalibrationSamples;
   zOffset /= nCalibrationSamples;
};

void Gyro::UpdateGyroReading(){
  gyro.read();
  
  xRaw = gyro.g.x;
  yRaw = gyro.g.y;
  zRaw = gyro.g.z;
    
  xContainer->AddValue(xRaw);
  yContainer->AddValue(yRaw);
  zContainer->AddValue(zRaw);
};

float Gyro::GetAngularSpeedAvg(){

  float gyroAngularSpeed = (zRaw - zOffset) * gyroSensitivity;

  // Convert to radian per second
  gyroAngularSpeed *= (2.*M_PI/360.0);

  return gyroAngularSpeed;
};

float Gyro::GetAngularSpeedRaw(){
  float gyroAngularSpeed = ( zContainer->GetAverage() - zOffset) * gyroSensitivity;

  // Convert to radian per second
  gyroAngularSpeed *= (2.*M_PI/360.0);

  return gyroAngularSpeed;
};


