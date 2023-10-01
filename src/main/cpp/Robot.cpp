// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "SwerveModule.h"

// std::thread t(&SwerveModule::run, &Robot::testModule);

void Robot::RobotInit()
{
  testModule.initMotors();
}
void Robot::RobotPeriodic()
{
  frc::SmartDashboard::PutNumber("SteerEnc", testModule.getSteerEncoder());
  frc::SmartDashboard::PutNumber("DriveEnc", testModule.getDriveEncoder());
}

void Robot::AutonomousInit()
{
  testModule.setSteerAngleSetpoint(25.0);
}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {}

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
