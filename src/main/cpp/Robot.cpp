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
  frc::SmartDashboard::PutNumber("Gyro", mGyro.getBoundedAngle() * 180 / M_PI);
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
  mHeadingController.setHeadingControllerState(SwerveHeadingController::SNAP);
}
void Robot::TeleopPeriodic()
{
  
  // frc::SmartDashboard::PutNumber("GYRO", mGyro.getBoundedAngle());
  // frc::SmartDashboard::PutNumber("leftY", ctr.GetLeftY());
  double rot = ControlUtil::deadZoneQuadratic(ctr.GetRightX(), ctrDeadzone);
  frc::SmartDashboard::PutNumber("POV", ctr.GetPOV());

  if (ctr.GetPOV() > 0) {
    mHeadingController.setSetpoint(ctr.GetPOV());
    
  }

  double hc = mHeadingController.update(mGyro.getBoundedAngle() * 180 / M_PI);

  if (ctr.GetAButton()) {
    // rot = mHeadingController.update(mGyro.getBoundedAngle() * 180 / M_PI);
    rot = hc;
  }
  frc::SmartDashboard::PutNumber("HC", hc);


  

  // frc::SmartDashboard::PutNumber("Rotval", rot);
  frc::SmartDashboard::PutNumber("Rot", rot);
  mDrive.Drive(
    rot, 
    ControlUtil::deadZoneQuadratic(ctr.GetLeftX() / 2, ctrDeadzone), 
    ControlUtil::deadZoneQuadratic(-ctr.GetLeftY() / 2, ctrDeadzone), 
    mGyro.getBoundedAngle());
  mDrive.displayDriveTelemetry();

  if (ctr.GetAButtonReleased()) {
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
