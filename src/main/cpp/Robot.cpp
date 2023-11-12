// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit()
{
  testModule.initMotors();
  // Instantiating pre-declared thread
  testThread = std::thread(&SwerveModule::run, &testModule);
}
void Robot::RobotPeriodic()
{
  // Print the steer encoder position and drive encoder velocity
  frc::SmartDashboard::PutNumber("Steer", testModule.getSteerEncoder().getRadians());
  frc::SmartDashboard::PutNumber("Velocity", testModule.getDriveEncoderVel());
}

void Robot::AutonomousInit()
{
  
  
}
void Robot::AutonomousPeriodic() {
  testModule.exitStandbyThread();
  testModule.setDriveVelocitySetpoint(20); // RPM
  testModule.setSteerAngleSetpoint(0.3);
}

void Robot::TeleopInit()
{
  
  testModule.exitStandbyThread();
  testModule.setDriveVelocitySetpoint(0.0);
  testModule.setSteerAngleSetpoint(0.0);
}
void Robot::TeleopPeriodic() {
  startingPos += ctr->GetRightX();
  testModule.setSteerAngleSetpoint(startingPos);
  testModule.setDriveVelocitySetpoint(ctr->GetLeftY());
}

void Robot::DisabledInit()
{
  //Put module in standby(stop motors and wait)
  testModule.standbyThread();
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
