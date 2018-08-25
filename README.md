# Udacity Flying Car Nano Degree Quadrotor PID Controller #
#### Derek Lukacs ####
This repository is a branch and add on to a project from the Udacity Flying Car Nano Degree (FCND) program. 

<img src="https://github.com/dereklukacs/FCND-Controller/blob/master/images/Udacity_P3_manyquads.gif?raw=true"
		 alt="Simulation of any quads following a figure 8 trajectory"/>
## Overview ##

This diagram is similar to the control diagram but focuses on the actual implementation. The implementation inside of the blue blocks is what was done for this project. By defining the behavior inside these blocks the control behavior is defined.

<img src="https://github.com/dereklukacs/FCND-Controller/blob/master/images/information-flow-overview.png?raw=true"
     alt="Controller information flow chart" />

The trajectory is generated offline and read in from a configuration file. The State is calculated and assumed ideal for the purposes of creating the controller. The later [FCND project on estimation](https://github.com/dereklukacs/FCND-Estimation-CPP) removes this assumption of perfect state and builds a state estimate with sensor data and an Extended Kalman Filter.

Various simulations and scenarios were used for this project to tune the PID gains on the various control loops. The vehicle was tuned from the inner loops top the outer loops. 

## Calculating Motor Thrusts ##

The most inner loop of this controller is calculating motor thrusts from the altitude controller and body rate controller. This is sometimes referred to as "motor mixing." It is done by solving a system of 4 linear equations for the individual motor thrusts. 

<img src="https://github.com/dereklukacs/FCND-Controller/blob/master/images/Drone1.png?raw=true"
     alt="diagram of quad with labeled motors and axes"
     width=400px/>

<img src="https://github.com/dereklukacs/FCND-Controller/blob/master/images/motor_mixing_equations.png?raw=true"
		alt="motor mixing system of equations"
     width=400px/>

In the above system F is the total thrust which is calculated by the altitude controller. The other parameters are the body torques calculated by the body rate controller. L and kappa are parameters from dynamics of the quad. L is the arm length of the quadrotor and kappa is the ratio of thrust to drag on the propellers. The drag on the propellers is what produces a yaw torque. 

Solving these equations for the torques results in the following solution.

<img src="https://github.com/dereklukacs/FCND-Controller/blob/master/images/motor_mixing_solutions.png?raw=true"
		alt="Motor torque solutions"
     width=350px/>

## Body Rate Control ##
The body rate controller in this project takes three desired angular rates as well as the current angular rates to calculate a moment to induce. This is done with a proportional term on the error between PQR<sub>actual</sub> and PQR<sub>desired</sub>. Additionally, in order to take into consideration the dynamics of the vehicle, the inertial vector is factored in.

This results in a controller that controls the angular rates to the desired values depending on outer loop controllers such as `Yaw()` and `RollPitch()`.

The simulation below was used to tune the gains on the body rate controller. The quad in this simulation had initial angular rates in roll and the control nulled these rates to zero. 

<img src="https://github.com/dereklukacs/FCND-Controller/blob/master/images/Udacity_P3_scen2.gif?raw=true"
		 alt="Simulation of quad cancelling angular rates to zero"/>

## Roll Pitch Control ##

In order to control the roll and pitch the body rates must be controlled. To calculate the body rate commands it is necessary to calculate the roll and pitch rates in the inertial frame and then do a coordinate transformation into the body frame. Roll and pitch rates come from a proportional term on the roll pitch error.

The simulation shown in the Body Rate Control section was also used to tune the gains on roll pitch controller after the body rate controller had been tuned. 

## Yaw Control ##
The yaw controller calculates the desired yaw rate based on the current yaw error and a proportional term. This is a first order system.

## Lateral Position Control ##
The lateral position controller calculates the necessary horizontal acceleration to control the XY position. This then gets fed into the roll pitch controller which seeks to achieve this acceleration. The lateral acceleration is calculated with proportional and derivative terms. The acceleration command is then constrained by ± maxAccelXY. This is a second order control loop.

The position control loop was tuned and tested with a simulation that had 2 quads moving 1 meter. One of the quads was off by 45 degrees in yaw and this was corrected with the yaw controller. 

<img src="https://github.com/dereklukacs/FCND-Controller/blob/master/images/Udacity_P3_scen3.gif?raw=true"
		 alt="Simulation of 2 quads moving 1 meter"/>

## Altitude Control ##
The altitude controller calculates the necessary total thrust to control the vertical position of the vehicle. Error for both proportional corrections and derivative corrections are applied. Any acceleration commands can be passed into the controller as a feed forward term. The addition of an integral term assists with errors in the mass model of the vehicle.

The P, I, and D terms on the altitude controller were tuned with a simulation that involved three quadrotors. One had an off-center center of mass while another had a mass higher than what was modeled. Without the I term on altitude the quad with the red path would not finish at the proper altitude.

<img src="https://github.com/dereklukacs/FCND-Controller/blob/master/images/Udacity_P3_scen4.gif?raw=true"
		 alt="Simulation of 3 quads moving with various mass properties."/>

## Precision tuning ##

The final tuning was done on a tricky to follow figure eight trajectory. 

<img src="https://github.com/dereklukacs/FCND-Controller/blob/master/images/Udacity_P3_scen5.gif?raw=true"
		 alt="Figure eight trajectory used for final tuning"/>

For this path a feed forward velocity was included as part of the trajectory and this greatly improves the accuracy with which the quads follow the trajectory. Without the feed forward term there is a time delay because there needs to be a high enough proportional error to make up for the difference in velocity.

## Conclusions ##
This work was done to learn by doing many concepts taught in the Udacity FCND program. It resulted in a controller that is capable of following arbitrary trajectories in 3D space with perfect state information. The next level of complication is to take this controller and use real time state estimation methods to use as the state input to the controller. This is done in the next project completed, the [FCND project on estimation](https://github.com/dereklukacs/FCND-Estimation-CPP).
