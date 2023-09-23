// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit()
{
  //mDrive.initAllMotors();
  testModule.initMotors();
}
void Robot::RobotPeriodic() {
  frc::SmartDashboard::PutNumber("SteerEnc", testModule.getSteerEncoder());
  frc::SmartDashboard::PutNumber("DriveEnc", testModule.getDriveEncoder());
}

void Robot::AutonomousInit() {
}
void Robot::AutonomousPeriodic() {
}

void Robot::TeleopInit() {
  testModule.setSteerAngleSetpoint(10);
}
void Robot::TeleopPeriodic()
{
  
  
  //double gyro = 0;
  //mDrive.Drive(mController->GetRightX(), mController->GetLeftX(), mController->GetLeftY(), gyro);
}

void Robot::DisabledInit() {}
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
