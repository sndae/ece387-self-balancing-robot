# Introduction #
### general scope ###
To build our robot we started by researching the implementation of the embedded chips we had. The data from the chips was acquired via I2C protocol. We used the sensory output of accelerometer ADX and gyroscope X to provide stabilization. It was used to calculate the instantaneous angle difference from the desired upright position. The position was corrected in software with Kalman filter methodology. The angle offset determined speed control to bring the robot to the upright position.
## Marketability ##
A balancing device such as this can be used as the foundation for several applications.
  * An assistive balncing robot can be used to help a single person carry a heavy object by holding up half of the object's load.  For example, a person carrying a heavy log could put one side of the log on top of the robot and carry the other end of the log.  The person could control the robot's speed by tilting the log slightly upward or downward.

  * A large balancing robot can be used as a 2-wheeled personal transportation device, such as a balancing skateboard, [Segway](http://www.segway.com/), unicycle, or even a [wheelchair](http://www.youtube.com/watch?v=Iv_SfonG4w4)

  * Self-balancing two-wheeled robots tend to due exceptionally well over rough terrain.  A radio-controlled self-balancing robot could be implemented to investigate uncharted sections of terrain while maintaining the safety of the operator.

## Time Spent for Design and Implementation ##
  * 3 hours----coming up with the physical design of the robot by doing a verbal feasibility assessment of potential design prototypes and making a parts list
  * 7 hours----shopping time and physical build.  This included time spent on slight modifications when parts didn't fit as anticipated
  * 40 hours--Code drafting, implementation, and troubleshooting.  The majority of this time was spent figuring out the best algorithm based on the gyroscope and accelerometer readings to balance the robot.
Total: 50 hours
## Kalman Filter ##
The Kalman filter is a linear estimator used to describe the future state of some processed based on the most recent state and most reliable prior states. While the process of balancing the robot may not be best described as linear, it was known that other designs had successfully adopted this approach to remove the jarring affect on the accelerometer during direction changes to provide better indication of position based on accelerometer readings.
## Inverted Pendulum ##

An inverted pendulum is a weighted object which has its center of mass above its pivot point.  Normal pendulums are stable at rest, hanging downwards from some pivot above.  Inverted pendulums are essentially unstable and must be actively balanced by either applying torque at the pivot point or by moving the pivot point horizontally as part of a feedback system, like in our application.
## PID control ##

PID control was developed because the real-time discrepancy from a given setpoint alone (the P of PID) was not data enough to provide for an adequate correction of a dynamic process. The rate of change in error (D of PID)and the accumulation of previous errors (I of PID) can play a role in providing a control signal, and if that data is available it should be used. How it is used is the basis for PID control.

The general formula for a correction signal using PID control is as follows:

S = kp x Error + ki x Sum of Errors + kd x dError/dt

The kp, ki, and kd multipliers are proportional constants used to emphasize parts of the equation.