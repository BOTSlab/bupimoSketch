#include <Wire.h>
#include <Zumo32U4.h>

class Encoders {
  public:
    Encoders(float minTime = 100., float bl = 0.085, float wr = 0.019, float cpr = 1204.44);

    void UpdateWheelSpeeds();

    float GetLeftWheelSpeed();
    float GetRightWheelSpeed();

    float GetAngularSpeed();
    float GetLinearSpeed();
  private:

    // Wheel speeds as measured by the encoders. Measured in rads/sec.
    float rightWheelSpeed;
    float leftWheelSpeed;

    // These values are calculated mased n the zumos dimensions and the measured wheel speeds
    float linearSpeed;
    float angularSpeed;

    // This is the minimum time (in milliSeconds) between updates of the wheel encoders
    float minDeltaTime;

    // Used to measure the elapsed time;
    unsigned long lastTimeStamp;

    // These values are all set by the physical dimensions of the Zumo
    float baseLength;
    float wheelRadius;
    float countsPerRotation;

    Zumo32U4Encoders encoders;
};

Encoders::Encoders(float minTime, float bl, float wr, float cpr) {
  rightWheelSpeed = 0.;
  leftWheelSpeed = 0.;
  linearSpeed = 0.;
  angularSpeed = 0.;

  baseLength = bl;
  wheelRadius = wr;
  countsPerRotation = cpr;
};

void Encoders::UpdateWheelSpeeds() {

  // number of milliseconds since last timeStamp
  unsigned long delta = millis() - lastTimeStamp;

  // Change to number of milliseconds
  float deltaT = (float)(delta);

  // Read how many counts of occured since the last time
  int16_t countsLeft = encoders.getCountsAndResetLeft();
  int16_t countsRight = encoders.getCountsAndResetRight();

  // I will need to figure out how to use these error flags
  //bool errorLeft = encoders.checkErrorLeft();
  //bool errorRight = encoders.checkErrorRight();

  // Calculate the angular speed of the right and left wheels in units of [rads/s]
  float cR = (float)(countsRight);
  float cL = (float)(countsLeft);

  rightWheelSpeed = (cR / countsPerRotation) * 2. * M_PI / (deltaT / 1000.);
  leftWheelSpeed = (cL / countsPerRotation) * 2. * M_PI / (deltaT / 1000.);

  lastTimeStamp = millis();

}

float Encoders::GetLeftWheelSpeed() {
  return leftWheelSpeed;
};

float Encoders::GetRightWheelSpeed() {
  return rightWheelSpeed;
};

float Encoders::GetLinearSpeed() {
  linearSpeed = wheelRadius * 0.5 * (leftWheelSpeed + rightWheelSpeed);
  return linearSpeed;
};

float Encoders::GetAngularSpeed() {
  angularSpeed = wheelRadius / baseLength * (rightWheelSpeed - leftWheelSpeed);
  return angularSpeed;
};

