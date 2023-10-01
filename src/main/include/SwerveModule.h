#pragma once

#include <rev/CANSparkMax.h>
#include <thread>
#include "Translation2d.h"
#include "SwerveModuleState.h"
#include "Constants.h"

class SwerveModule
{
// private: <- removed for testing
public:
    int steerID = 7;
    int driveID = 17;
    rev::CANSparkMax *steerMotor  = new rev::CANSparkMax(steerID, rev::CANSparkMax::MotorType::kBrushless);
    rev::CANSparkMax *driveMotor = new rev::CANSparkMax(driveID, rev::CANSparkMax::MotorType::kBrushless);
    

    rev::SparkMaxPIDController steerPID = steerMotor->GetPIDController();
    rev::SparkMaxPIDController drivePID = driveMotor->GetPIDController();

    rev::SparkMaxRelativeEncoder steerEnc = steerMotor->GetEncoder();
    rev::SparkMaxRelativeEncoder driveEnc = driveMotor->GetEncoder();

    double steerP = 0.3;
    double steerI = 0;
    double steerD = 0;
    double driveP = 0.3;
    double driveI = 0;
    double driveD = 0;

    float driveVelocitySetpoint;
    float drivePositionSetpoint;
    float steerAngleSetpoint;
    bool driveModePosition = false;
    bool stopThread = false;



// public:
    SwerveModule(int steerMotorID, int driveMotorID);
    void initMotors();
    float getSteerAngleSetpoint();
    void setSteerAngleSetpoint(float setpt);
    void setSteerAngleSetpointShortestPath(float setpt);
    void setDrivePositionSetpoint(float setpt);
    void setDriveVelocitySetpoint(float setpt);
    void setModuleState(SwerveModuleState setpt);
    double getSteerEncoder();
    double getDriveEncoder();
    bool isFinished(float percentageBound);
    void run(); // Not working, not ready to thread yet
    void joinThread(); // Not working, not ready to thread yet



};