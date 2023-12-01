// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include "ShuffleUI.h"
#include <thread>
#include "SwerveDrive.h"
//#include "SwerveModule.h"

class Robot : public frc::TimedRobot
{
public:
  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  void SimulationInit() override;
  void SimulationPeriodic() override;

  frc::XboxController *ctr = new frc::XboxController(0);
  SwerveDrive mDrive = SwerveDrive();
  

  // Test code below that I'm leaving in case I need it later
  //SwerveModule testModule = SwerveModule(4, 16);
  //SwerveModule testModule = SwerveModule(1, 42, 0);
  //SwerveModule testModule3 = SwerveModule(11, 18);
  //std::thread testThread;
  //std::thread testThread2;
  //std::thread testThread3;
};
