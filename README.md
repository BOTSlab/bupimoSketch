The Arduino sketch for our robots which use the Pololu Zumo 32U4 base. The
sketch listens for commands as well as outputs sensor readings over serial. The
commands need the following form:

v:w:r

Where v is the linear speed (m/s), w is the angular speed (rads/s), and
r is a 0/1 value indicating whether the odometric estimate of position should
be reset to (0, 0, 0).  The speed values commands are compared to values from
the encoders by a low-level controller which then controls the motors.

3 seconds after receiving a command the robot will stop.

The output data has the form:

encoder -> linear speed

gyro -> angular speed

encoder -> angular speed

compass -> heading (currently useless)

prox. sensors -> counts from front detector

------------------------------------------------------------------------
The Arduino sketch required for the Bupimo robot.

29 May, 2018
Andrew Vardy
