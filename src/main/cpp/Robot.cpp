// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit()
{
  testModule.initMotors();
  testThread = std::thread(&SwerveModule::run, &testModule); // Instantiating pre-declared thread
  testModule2.initMotors();
  testThread2 = std::thread(&SwerveModule::run, &testModule2); // Instantiating pre-declared thread
  testModule3.initMotors();
  testThread3 = std::thread(&SwerveModule::run, &testModule3); // Instantiating pre-declared thread
}
void Robot::RobotPeriodic()
{
  // Print the steer encoder position and drive encoder velocity
  frc::SmartDashboard::PutNumber("Steer", testModule.getSteerEncoder().getRadians());
  frc::SmartDashboard::PutNumber("Velocity", testModule.getDriveEncoderVel());
}

void Robot::AutonomousInit()
{
  testModule.exitStandbyThread();
  testModule2.exitStandbyThread();
  testModule3.exitStandbyThread(); // Start running motor threads
}
void Robot::AutonomousPeriodic()
{
  testModule.setDriveVelocitySetpoint(100); // RPM
  testModule.setSteerAngleSetpoint(0.3);
  testModule2.setDriveVelocitySetpoint(100); // RPM
  testModule2.setSteerAngleSetpoint(0.3);
  testModule3.setDriveVelocitySetpoint(100); // RPM
  testModule3.setSteerAngleSetpoint(0.3);
}
void Robot::TeleopInit()
{
  testModule.exitStandbyThread(); // Start running motor threads
  testModule.setDriveVelocitySetpoint(0.0);
  testModule.setSteerAngleSetpoint(0.0);
  testModule2.exitStandbyThread(); // Start running motor threads
  testModule2.setDriveVelocitySetpoint(0.0);
  testModule2.setSteerAngleSetpoint(0.0);
  testModule3.exitStandbyThread(); // Start running motor threads
  testModule3.setDriveVelocitySetpoint(0.0);
  testModule3.setSteerAngleSetpoint(0.0);
}
void Robot::TeleopPeriodic()
{
  // Increment the steer motor angle by our controller's right X value
  double angle = atan2(-ctr->GetLeftY(), ctr->GetLeftX());
  double mag = sqrt((ctr->GetLeftX()*ctr->GetLeftX()) + (ctr->GetLeftY()*ctr->GetLeftY()));

  frc::SmartDashboard::PutNumber("JAngle", angle);
  // Step 1: Subtract by pi/2 to change from polar to 0 north
  angle -= M_PI_2;
  angle += PI * 2;


  // Step 2: Modulus to change from 0 - pi to 0 - 2pi
  angle = fmod(angle, M_PI * 2);
  if (fabs(mag) < 0.01) {
    angle = 0.0;
  }
  
  frc::SmartDashboard::PutNumber("InAngle", angle);
  testModule.setSteerAngleSetpoint(angle*3.375);
  testModule.setDrivePercentVelocitySetpoint(mag);
  testModule2.setSteerAngleSetpoint(angle*3.375);
  testModule2.setDrivePercentVelocitySetpoint(mag);
  testModule3.setSteerAngleSetpoint(angle*3.375);
  testModule3.setDrivePercentVelocitySetpoint(mag);
}

void Robot::DisabledInit()
{
  // Put module in standby(stop motors and wait)
  testModule.standbyThread();
  testModule2.standbyThread();
  testModule3.standbyThread();
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
