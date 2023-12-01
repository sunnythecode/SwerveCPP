// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit()
{
  testModule.initMotors();
  testThread = std::thread(&SwerveModule::run, &testModule);
  //mDrive.initAllMotors();
}
void Robot::RobotPeriodic()
{

}

void Robot::AutonomousInit()
{
  testModule.exitStandbyThread();
  //mDrive.enableThreads();
}
void Robot::AutonomousPeriodic()
{

}
void Robot::TeleopInit()
{
  testModule.exitStandbyThread();
  //mDrive.enableThreads();
}
void Robot::TeleopPeriodic()
{
  double mag = sqrt((ctr->GetLeftY() * ctr->GetLeftY()) + (ctr->GetLeftX() * ctr->GetLeftX()));
  double angle = atan2(-ctr->GetLeftY(), ctr->GetLeftX());

  angle = -angle + M_PI_2;
  angle = angle * 180 / M_PI;
  angle = fmod(angle + 360, 360);
  angle = angle * M_PI / 180;

  frc::SmartDashboard::PutNumber("Mag", mag);
  frc::SmartDashboard::PutNumber("Angle_Deg", angle * 180 / M_PI);

  if (fabs(mag) > 0.15) {
    testModule.setDrivePercentVelocitySetpoint(mag);
    testModule.setSteerAngleSetpoint(angle);
  }
  else {
    testModule.setDrivePercentVelocitySetpoint(0);
  }

  frc::SmartDashboard::PutNumber("EncoderRadians", testModule.getSteerEncoder().getDegrees());

  

  // Swerve Drive function
  //mDrive.Drive(ctr->GetRightX(), ctr->GetLeftX(), -ctr->GetLeftY(), 0.0);
}

void Robot::DisabledInit()
{
  testModule.standbyThread();
  //mDrive.stopAllMotors();
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
