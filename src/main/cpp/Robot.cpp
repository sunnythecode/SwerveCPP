// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit()
{
  testModule.initMotors();
  testThread = std::thread(&SwerveModule::run, &testModule); // Instantiating pre-declared thread
}
void Robot::RobotPeriodic()
{
  // Print the steer encoder position and drive encoder velocity
  frc::SmartDashboard::PutNumber("Steer", testModule.getSteerEncoder().getRadians());
  frc::SmartDashboard::PutNumber("Velocity", testModule.getDriveEncoderVel());
}

void Robot::AutonomousInit()
{
  testModule.exitStandbyThread(); // Start running motor threads
}
void Robot::AutonomousPeriodic()
{
  testModule.setDriveVelocitySetpoint(100); // RPM
  testModule.setSteerAngleSetpoint(0.3);
}
void Robot::TeleopInit()
{
  testModule.exitStandbyThread(); // Start running motor threads
  testModule.setDriveVelocitySetpoint(0.0);
  testModule.setSteerAngleSetpoint(0.0);
}
void Robot::TeleopPeriodic()
{
  // Increment the steer motor angle by our controller's right X value
  testModule.setSteerAngleSetpoint(testModule.getSteerAngleSetpoint() + ctr->GetRightX());
  // Set our drive motor PID setpoint to maxRPM * left Y value
  testModule.setDrivePercentVelocitySetpoint(ctr->GetLeftY());
}

void Robot::DisabledInit()
{
  // Put module in standby(stop motors and wait)
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
