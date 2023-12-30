#pragma once

#include <thread>
#include <string>

#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/SparkMaxPIDController.h>
#include <rev/CANSparkMax.h>

#include "CAN_Coder.h"
#include "Translation2d.h"
#include "SwerveModuleState.h"
#include "Constants.h"

class SwerveModule
{
    // private: <- removed for testing
public:
    int steerID;
    int driveID;

    rev::CANSparkMax *steerMotor;
    rev::CANSparkMax *driveMotor;

    CAN_Coder steerEnc;
    rev::SparkMaxRelativeEncoder driveEnc;

    // PID Controller for Steer Motor
    frc2::PIDController steerCTR{steerP, steerI, steerD};

    // REV Default Velocity PID values(Drive Motor)
    double kP = revkP, kI = revkI, kD = revkD, kIz = revkIz, kFF = revkFF, kMaxOutput = revkMaxOutput, kMinOutput = revkMinOutput;

    // PID Controller for Drive Motor
    rev::SparkMaxPIDController m_pidController;

    float driveVelocitySetpoint;
    float drivePositionSetpoint;
    float steerAngleSetpoint;
    bool driveModePosition = false;
    bool stopThread = false;
    const int maxRPMFreeSpeed = moduleMaxRPM;

    // public:
    SwerveModule(int steerMotorID, int driveMotorID, int CAN_ID);
    void initMotors();
    float getSteerAngleSetpoint();
    void setSteerAngleSetpoint(float setpt);
    bool setSteerAngleSetpointShortestPath(float setpt);
    void setDrivePositionSetpoint(float setpt);
    void setDriveVelocitySetpoint(float setpt);
    void setDrivePercentVelocitySetpoint(float setpt);
    void setModuleState(SwerveModuleState setpt);
    Rotation2d getSteerEncoder();
    double getDriveEncoderVel();
    double getDriveEncoderPos();
    bool isFinished(float percentageBound);
    void run();
    void standbyThread();
    void exitStandbyThread();
};