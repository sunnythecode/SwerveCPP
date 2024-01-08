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
  frc::SmartDashboard::PutNumber("Gyro", mGyro.getBoundedAngle().getDegrees());
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
  double rot = ControlUtil::deadZoneQuadratic(ctr.GetRightX(), ctrDeadzone);
  double rot2;
  frc::SmartDashboard::PutNumber("POV", ctr.GetPOV());

  if (ctr.GetAButton() && (ctr.GetPOV() > 0)) 
  {
    double rot2 = -mHeadingController.calculate(mGyro.getBoundedAngle().getDegrees(), Rotation2d::degreesBound(-ctr.GetPOV()));
  }

  frc::SmartDashboard::PutNumber("HC", rot2);


  // Swerve Drive
  mDrive.Drive(
    rot, 
    ControlUtil::deadZoneQuadratic(ctr.GetLeftX() / 2, ctrDeadzone), 
    ControlUtil::deadZoneQuadratic(-ctr.GetLeftY() / 2, ctrDeadzone), 
    mGyro.getBoundedAngle().getRadians());
  mDrive.displayDriveTelemetry();



  // Gyro Resets
  if (ctr.GetAButtonReleased()) 
  {
    mGyro.init();
  }

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
