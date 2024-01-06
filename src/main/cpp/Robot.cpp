// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit()
{
  mDrive.initAllMotors();
  mGyro.init();
}
void Robot::RobotPeriodic()
{
  //ShuffleUI::MakeWidget("Gyro", driveTab, mGyro.getBoundedAngle(), frc::BuiltInWidgets::kGyro);
  frc::SmartDashboard::PutNumber("Gyro", mGyro.getBoundedAngle());
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
  
  // frc::SmartDashboard::PutNumber("GYRO", mGyro.getBoundedAngle());
  // frc::SmartDashboard::PutNumber("leftY", ctr.GetLeftY());
  mDrive.Drive(
    ControlUtil::deadZoneQuadratic(ctr.GetRightX(), ctrDeadzone), 
    ControlUtil::deadZoneQuadratic(ctr.GetLeftX() / 2, ctrDeadzone), 
    ControlUtil::deadZoneQuadratic(-ctr.GetLeftY() / 2, ctrDeadzone), 
    mGyro.getBoundedAngle());
  mDrive.displayDriveTelemetry();
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
