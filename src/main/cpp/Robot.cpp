// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit()
{
  mDrive.initAllMotors();
}
void Robot::RobotPeriodic()
{

}

void Robot::AutonomousInit()
{
  mDrive.enableThreads();
}
void Robot::AutonomousPeriodic()
{

}
void Robot::TeleopInit()
{
  mDrive.enableThreads();
}
void Robot::TeleopPeriodic()
{
  // Swerve Drive function
  mDrive.Drive(ctr->GetRightX(), ctr->GetLeftX(), -ctr->GetLeftY(), 0.0);
}

void Robot::DisabledInit()
{
  mDrive.stopAllMotors();
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
