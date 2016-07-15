#include <Wire.h>
#include <Zumo32U4.h>

class EncoderOdometry {
  public:
    EncoderOdometry(float minTime = 100., float bl = 0.085, float wr = 0.019, float cpr = 1204.44);

    void Update();

    float GetX();
    float GetY();
    float GetTheta();

    float GetLeftWheelSpeed();
    float GetRightWheelSpeed();

    float GetAngularSpeed();
    float GetLinearSpeed();
  private:

    // Pose of the robot in the metres (x, y) and radians (theta).
    float x, y, theta;

    // Wheel speeds as measured by the encoders. Measured in rads/sec.
    float rightWheelSpeed;
    float leftWheelSpeed;

    // Displacements of the wheels.  Measured in the same units as the wheel radius
    // is specified in (metres).
    float rightWheelDisplacement;
    float leftWheelDisplacement;    

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

EncoderOdometry::EncoderOdometry(float minTime, float bl, float wr, float cpr) {
  x = 0;
  y = 0;
  theta = 0;
  rightWheelSpeed = 0.;
  leftWheelSpeed = 0.;
  rightWheelDisplacement = 0.;
  leftWheelDisplacement = 0.;
  linearSpeed = 0.;
  angularSpeed = 0.;

  baseLength = bl;
  wheelRadius = wr;
  countsPerRotation = cpr;
};

void EncoderOdometry::Update() {

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

  // Calculate the angular speeds of the right and left wheels in units of [rads/s]
  rightWheelSpeed = (countsRight / countsPerRotation) * 2. * M_PI / (deltaT / 1000.);
  leftWheelSpeed = (countsLeft / countsPerRotation) * 2. * M_PI / (deltaT / 1000.);

  // If we want the displacement (i.e. the distance travelled) by each wheel, then 
  // the calculation is the same, except that we must multiply by the wheel radius 
  // to get a distance and we don't divide by time
  rightWheelDisplacement = wheelRadius * (countsRight / countsPerRotation) * 2. * M_PI;
  leftWheelDisplacement = wheelRadius * (countsLeft / countsPerRotation) * 2. * M_PI;

  // Using the odometry model described on pages 270-271 of "Introduction to Autonomous
  // Mobile Robots", Second Edition, by Siegwart, Nourbakhsh, and Scaramuzza, 2011.
  float deltaTheta = (rightWheelDisplacement - leftWheelDisplacement) / baseLength;
  float deltaForward = (rightWheelDisplacement + leftWheelDisplacement) / 2.;
  x += deltaForward * cos(theta + deltaTheta/2.);
  y += deltaForward * sin(theta + deltaTheta/2.);
  theta += deltaTheta;

  lastTimeStamp = millis();
}

float EncoderOdometry::GetX() {
  return x;
}

float EncoderOdometry::GetY() {
  return y;
}

float EncoderOdometry::GetTheta() {
  return theta;
}

float EncoderOdometry::GetLeftWheelSpeed() {
  return leftWheelSpeed;
};

float EncoderOdometry::GetRightWheelSpeed() {
  return rightWheelSpeed;
};

float EncoderOdometry::GetLinearSpeed() {
  linearSpeed = wheelRadius * 0.5 * (leftWheelSpeed + rightWheelSpeed);
  return linearSpeed;
};

float EncoderOdometry::GetAngularSpeed() {
  angularSpeed = wheelRadius / baseLength * (rightWheelSpeed - leftWheelSpeed);
  return angularSpeed;
};

