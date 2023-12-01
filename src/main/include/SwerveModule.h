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
    frc2::PIDController steerCTR{0.2, 0.0, 0.0};

    // REV Default Velocity PID values(Drive Motor)
    double kP = 6e-5, kI = 1e-6, kD = 0, kIz = 0, kFF = 0.000015, kMaxOutput = 1.0, kMinOutput = -1.0;

    // PID Controller for Drive Motor
    rev::SparkMaxPIDController m_pidController;

    float driveVelocitySetpoint;
    float drivePositionSetpoint;
    float steerAngleSetpoint;
    bool driveModePosition = false;
    bool stopThread = false;
    const int maxRPMFreeSpeed = 5700;

    // public:
    SwerveModule(int steerMotorID, int driveMotorID, int CAN_ID); // To be implemented, unable to initialize motors here
    void initMotors();
    float getSteerAngleSetpoint();
    void setSteerAngleSetpoint(float setpt);
    void setSteerAngleSetpointShortestPath(float setpt);
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