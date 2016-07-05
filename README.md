-------------------------------------------------------------------------
UPDATE: July 5th 2016 Nicholi.

The Arduino sketch for the Bupimo robot. The sketch listens for commands
as well as outputs sensor readings over serial. The commands need the
following form:

v:w:t

Where v is the linear speed (m/s), w is the angular speed (rads/s), and
t is the length of time this command should be executed (s). A t value
of -1 will cause the command to be run indefinitely. These commands are 
compared to values from the encoder by a low level controller which then
controls the motors.

The output data has the form:
encoder -> linear speed
gyro -> angular speed
encoder -> angular speed
compass -> heading (currently useless)
prox. sensors -> counts from front detector.

------------------------------------------------------------------------
The Arduino sketch required for the Bupimo robot.

15 June, 2016
Andrew Vardy
----------------------------------------------------------------------