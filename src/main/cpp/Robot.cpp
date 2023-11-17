// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit()
{
  mSwerve.initAllMotors();
}
void Robot::RobotPeriodic()
{
}

void Robot::AutonomousInit()
{
  mSwerve.enableMotors();
}
void Robot::AutonomousPeriodic()
{
}
void Robot::TeleopInit()
{
  mSwerve.enableMotors();
}
void Robot::TeleopPeriodic()
{
  // // Increment the steer motor angle by our controller's right X value
  // testModule.setSteerAngleSetpoint(testModule.getSteerAngleSetpoint() + ctr->GetRightX());
  // // Set our drive motor PID setpoint to maxRPM * left Y value
  // testModule.setDrivePercentVelocitySetpoint(ctr->GetLeftY());

  mSwerve.Drive(ctr->GetRightX(), ctr->GetLeftX(), ctr->GetLeftY(), 0.0);

}

void Robot::DisabledInit()
{
  // Put module in standby(stop motors and wait)
  mSwerve.disableMotors();
}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
