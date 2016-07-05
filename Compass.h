#include "ContainerInt.h"
#include <Zumo32U4.h>
#include <Wire.h>

class Compass{
 public:

  Compass(int runningAverageSampleSize = 50);
  void Calibrate(int nCalibrationSamples = 250);
  void UpdateCompassReading(); 

  float GetHeadingAvg();
  float GetHeadingRaw();

  float GetZAvg();

 private:
  ContainerInt* xContainer;
  ContainerInt* yContainer;
  ContainerInt* zContainer;

  int xMin, xMax;
  int yMin, yMax;
  int zMin, zMax;

  int xRaw, yRaw, zRaw;

  LSM303 compass;
};

Compass::Compass(int runningAverageSampleSize){
  // Initialize hardware
  compass.init();
  compass.enableDefault();
  compass.writeReg(LSM303::CTRL5, 0x10); // Output Data Rate (ODR) 0x10 = 50hz
  //compass.writeReg(LSM303::CTRL6, 0x00); // Full Scale (FS)

  //Create containers
  xContainer = new ContainerInt(runningAverageSampleSize);
  yContainer = new ContainerInt(runningAverageSampleSize);
  zContainer = new ContainerInt(runningAverageSampleSize);

  // Fill the running average containers.
  for(int i = 0; i <runningAverageSampleSize; i++){
        UpdateCompassReading();      
  }

  xMin = yMin = zMin = 1*pow(2, 15);
  xMax = yMax = zMax = -1*pow(2, 15);
}

void Compass::Calibrate(int nCalibrationSamples){
   for(int index = 0; index < nCalibrationSamples; index ++){
    // Take a reading of the magnetic vector and store it in compass.m
    UpdateCompassReading();

    xMin = min(xMin, xContainer->GetAverage());
    xMax = max(xMax, xContainer->GetAverage());
    
    yMin = min(yMin, yContainer->GetAverage());
    yMax = max(yMax, yContainer->GetAverage());
    
    zMin = min(zMin, zContainer->GetAverage());
    zMax = max(zMax, zContainer->GetAverage());
    
    delay(40);
   }
};

void Compass::UpdateCompassReading(){
  compass.read();
  
  xRaw = compass.m.x;
  yRaw = compass.m.y;
  zRaw = compass.m.z;
    
  xContainer->AddValue(xRaw);
  yContainer->AddValue(yRaw);
  zContainer->AddValue(zRaw);
};

float Compass::GetHeadingRaw(){
  
  float x_scaled = 2.*(float)(xRaw - xMin) / (xMax - xMin) - 1.0;
  float y_scaled = 2.*(float)(yRaw - yMin) / (yMax - yMin) - 1.0;

  float angle = atan2(y_scaled, x_scaled)*180. / M_PI;
 
  return angle;
};

float Compass::GetHeadingAvg(){
 
  float x_scaled = 2.*(float)(xContainer->GetAverage() - xMin) / (xMax - xMin) - 1.0;
  float y_scaled = 2.*(float)(yContainer->GetAverage() - yMin) / (yMax - yMin) - 1.0;
  
  float angle = atan2(y_scaled, x_scaled)*180. / M_PI;

  return angle; 
};

float Compass::GetZAvg(){
  float z_scaled = 2.*(float)(zContainer->GetAverage() - zMin) / (zMax - zMin) - 1.0;

  return z_scaled;
};


