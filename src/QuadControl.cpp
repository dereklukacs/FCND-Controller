#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"
#include <ostream>
#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

void QuadControl::Init()
{
  BaseController::Init();

  // variables needed for integral control
  integratedAltitudeError = 0;
    
#ifndef __PX4_NUTTX
  // Load params from simulator parameter system
  ParamsHandle config = SimpleConfig::GetInstance();
   
  // Load parameters (default to 0)
  kpPosXY = config->Get(_config+".kpPosXY", 0);
  kpPosZ = config->Get(_config + ".kpPosZ", 0);
  KiPosZ = config->Get(_config + ".KiPosZ", 0);
     
  kpVelXY = config->Get(_config + ".kpVelXY", 0);
  kpVelZ = config->Get(_config + ".kpVelZ", 0);

  kpBank = config->Get(_config + ".kpBank", 0);
  kpYaw = config->Get(_config + ".kpYaw", 0);

  kpPQR = config->Get(_config + ".kpPQR", V3F());

  maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
  maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
  maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);
  maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);

  maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);

  minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
  maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);
#else
  // load params from PX4 parameter system
  //TODO
  param_get(param_find("MC_PITCH_P"), &Kp_bank);
  param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  // Convert a desired 3-axis moment and collective thrust command to 
  //   individual motor thrust commands
  // INPUTS: 
  //   collThrustCmd: desired collective thrust [N]
  //   momentCmd: desired rotation moment about each axis [N m]
  // OUTPUT:
  //   set class member variable cmd (class variable for graphing) where
  //   cmd.desiredThrustsN[0..3]: motor commands, in [N]

  // HINTS: 
  // - you can access parts of momentCmd via e.g. momentCmd.x
  // You'll need the arm length parameter L, and the drag/thrust ratio kappa

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

    float t = collThrustCmd;
    float r = momentCmd.x;
    float p = momentCmd.y;
    float y = momentCmd.z;
    
    cmd.desiredThrustsN[0] = (1.0/4.0)*t + (1.0/4.0)*sqrt(2.0)*r/L+(1.0/4.0)*sqrt(2.0)*p/L-(1.0/4.0)*y/kappa; // front left
    cmd.desiredThrustsN[1] = (1.0/4.0)*t - (1.0/4.0)*sqrt(2.0)*r/L+(1.0/4.0)*sqrt(2.0)*p/L+(1.0/4.0)*y/kappa; // front right
    cmd.desiredThrustsN[2] = (1.0/4.0)*t + (1.0/4.0)*sqrt(2.0)*r/L-(1.0/4.0)*sqrt(2.0)*p/L+(1.0/4.0)*y/kappa; // rear left
    cmd.desiredThrustsN[3] = (1.0/4.0)*t - (1.0/4.0)*sqrt(2.0)*r/L-(1.0/4.0)*sqrt(2.0)*p/L-(1.0/4.0)*y/kappa; // rear right

    for(int i =0; i<4 ; ++i){
        cmd.desiredThrustsN[i] = CONSTRAIN(cmd.desiredThrustsN[i], minMotorThrust, maxMotorThrust);
    }
  /////////////////////////////// END STUDENT CODE ////////////////////////////
  return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  // Calculate a desired 3-axis moment given a desired and current body rate
  // INPUTS: 
  //   pqrCmd: desired body rates [rad/s]
  //   pqr: current or estimated body rates [rad/s]
  // OUTPUT:
  //   return a V3F containing the desired moments for each of the 3 axes

  V3F momentCmd;
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    V3F inertia_vec(Ixx, Iyy, Izz);
    momentCmd = inertia_vec*kpPQR*(pqrCmd - pqr);
  /////////////////////////////// END STUDENT CODE ////////////////////////////
  return momentCmd;
}


// returns a desired roll and pitch rate


// THINKING: determine angles needed to obtain accelCmd vector from collThrustCmd scalar.
// i.e. there is a vector along which applying the collective thrust produces the desired
// acceleration. Find the roll and pitch angles that result in this vector (in inertial frame)
// and transform these angles into the body frame. This gives angular errors to then calculate body rates.
// if I want to accel in x I need to PITCH about y, SohCahToa,
// collAccel = hypotenuse
// horizontal accel = opposite.. arcsin
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  // Calculate a desired pitch and roll angle rates based on a desired global
  //   lateral acceleration, the current attitude of the quad, and desired
  //   collective thrust command
  // INPUTS: 
  //   accelCmd: desired acceleration in global XY coordinates [m/s2]
  //   attitude: current or estimated attitude of the vehicle
  //   collThrustCmd: desired collective thrust of the quad [N]
  // OUTPUT:
  //   return a V3F containing the desired pitch and roll rates. The Z
  //     element of the V3F should be left at its default value (0)

  V3F pqrCmd;
  Mat3x3F R = attitude.RotationMatrix_IwrtB();

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

    // parse roll pitch angles
    float b_a_x = R(0,2);
    float b_a_y = R(1,2);
    //turn collThrust into m/s^2
    float collAccelCmd = -collThrustCmd / mass;
    //calculate target r,p angles
    float b_a_x_target = atan ( accelCmd.x / collAccelCmd);
    float b_a_y_target = atan ( accelCmd.y / collAccelCmd);
    //constrain angles
    b_a_x_target = CONSTRAIN(b_a_x_target, -maxTiltAngle, maxTiltAngle);
    b_a_y_target = CONSTRAIN(b_a_y_target, -maxTiltAngle, maxTiltAngle);
    // use propertional term to calculate desired rate
    float b_c_x_dot = kpBank* (b_a_x_target - b_a_x);
    float b_c_y_dot = kpBank* (b_a_y_target - b_a_y);
    // convert to body frame rates
    pqrCmd.x = 1.0/R(2,2) * (  R(1,0)*b_c_x_dot - R(0,0)* b_c_y_dot );
    pqrCmd.y = 1.0/R(2,2) * (  R(1,1)*b_c_x_dot - R(0,1)* b_c_y_dot );
    
  /////////////////////////////// END STUDENT CODE ////////////////////////////
  return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
  // Calculate desired quad thrust based on altitude setpoint, actual altitude,
  //   vertical velocity setpoint, actual vertical velocity, and a vertical 
  //   acceleration feed-forward command
  // INPUTS: 
  //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
  //   posZ, velZ: current vertical position and velocity in NED [m]
  //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
  //   dt: the time step of the measurements [seconds]
  // OUTPUT:
  //   return a collective thrust command in [N]

  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust = 0;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    velZCmd = CONSTRAIN(velZCmd, -maxAscentRate, maxDescentRate);

    float pos_err = posZCmd - posZ;
    float vel_err = velZCmd - velZ;

    integratedAltitudeError += dt*pos_err * KiPosZ;
    float z_accel = pos_err * kpPosZ + vel_err*kpVelZ + accelZCmd + integratedAltitudeError;
    thrust = (9.81 - z_accel) *mass / R(2,2);

  /////////////////////////////// END STUDENT CODE ////////////////////////////
  
  return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
  // Calculate a desired horizontal acceleration based on 
  //  desired lateral position/velocity/acceleration and current pose
  // INPUTS: 
  //   posCmd: desired position, in NED [m]
  //   velCmd: desired velocity, in NED [m/s]
  //   pos: current position, NED [m]
  //   vel: current velocity, NED [m/s]
  //   accelCmdFF: feed-forward acceleration, NED [m/s2]
  // OUTPUT:
  //   return a V3F with desired horizontal accelerations. 
  //     the Z component should be 0
  // HINTS: 
  //  - use the gain parameters kpPosXY and kpVelXY
  //  - make sure you limit the maximum horizontal velocity and acceleration
  //    to maxSpeedXY and maxAccelXY

  // make sure we don't have any incoming z-component
  accelCmdFF.z = 0;
  velCmd.z = 0;
  posCmd.z = pos.z;

  // we initialize the returned desired acceleration to the feed-forward value.
  // Make sure to _add_, not simply replace, the result of your controller
  // to this variable
  V3F accelCmd = accelCmdFF;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    V3F pos_err =  posCmd - pos;
    V3F vel_err =  velCmd - vel;
    
    accelCmd.x = accelCmd.x + pos_err.x * kpPosXY + vel_err.x * kpVelXY;
    accelCmd.y = accelCmd.y + pos_err.y * kpPosXY + vel_err.y * kpVelXY;
    accelCmd.x = CONSTRAIN(accelCmd.x, -maxAccelXY, maxAccelXY );
    accelCmd.y = CONSTRAIN(accelCmd.y, -maxAccelXY, maxAccelXY );
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return accelCmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
  // Calculate a desired yaw rate to control yaw to yawCmd
  // INPUTS: 
  //   yawCmd: commanded yaw [rad]
  //   yaw: current yaw [rad]
  // OUTPUT:
  //   return a desired yaw rate [rad/s]
  // HINTS: 
  //  - use fmodf(foo,b) to unwrap a radian angle measure float foo to range [0,b]. 
  //  - use the yaw control gain parameter kpYaw

  float yawRateCmd=0;
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    yaw = fmodf(yaw,2*3.14159);
    float yaw_err = yawCmd - yaw;
    yawRateCmd = yaw_err * kpYaw;
  /////////////////////////////// END STUDENT CODE ////////////////////////////
  return yawRateCmd;
}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  curTrajPoint = GetNextTrajectoryPoint(simTime);

  float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);

  // reserve some thrust margin for angle control
  float thrustMargin = .1f*(maxMotorThrust - minMotorThrust);
  collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust+ thrustMargin)*4.f, (maxMotorThrust-thrustMargin)*4.f);
  
  V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);
  
  V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
  desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

  V3F desMoment = BodyRateControl(desOmega, estOmega);

  return GenerateMotorCommands(collThrustCmd, desMoment);
}
