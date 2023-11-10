#pragma once

#include <rev/CANSparkMax.h>
#include <thread>
#include "Translation2d.h"
#include "SwerveModuleState.h"
#include "Constants.h"
#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>

class SwerveModule
{
    // private: <- removed for testing
public:
    int steerID = 7;
    int driveID = 17;
    rev::CANSparkMax *steerMotor = new rev::CANSparkMax(steerID, rev::CANSparkMax::MotorType::kBrushless);
    rev::CANSparkMax *driveMotor = new rev::CANSparkMax(driveID, rev::CANSparkMax::MotorType::kBrushless);

    rev::SparkMaxRelativeEncoder steerEnc = steerMotor->GetEncoder();
    rev::SparkMaxRelativeEncoder driveEnc = driveMotor->GetEncoder();

    frc2::PIDController driveCTR{0.01, 0.0, 0.0};
    frc2::PIDController steerCTR{0.5, 0.0, 0.0};

    double steerP = 0.3;
    double steerI = 0;
    double steerD = 0;
    double driveP = 6e-5;
    double driveI = 1e-6;
    double driveD = 0;
    double driveIz = 0;
    double driveFF = 0.000015;

    float driveVelocitySetpoint;
    float drivePositionSetpoint;
    float steerAngleSetpoint;
    bool driveModePosition = false;
    bool stopThread = false;

    // public:
    SwerveModule(int steerMotorID, int driveMotorID); // To be implemented, unable to initialize motors here
    void initMotors();
    float getSteerAngleSetpoint();
    void setSteerAngleSetpoint(float setpt);
    void setSteerAngleSetpointShortestPath(float setpt);
    void setDrivePositionSetpoint(float setpt);
    void setDriveVelocitySetpoint(float setpt);
    void setModuleState(SwerveModuleState setpt);
    Rotation2d getSteerEncoder();
    double getDriveEncoderVel();
    double getDriveEncoderPos();
    bool isFinished(float percentageBound);
    void run();
    void standbyThread();
    void exitStandbyThread();
};