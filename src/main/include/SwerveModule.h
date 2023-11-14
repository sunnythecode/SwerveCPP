#pragma once

#include <rev/CANSparkMax.h>
#include <thread>
#include "Translation2d.h"
#include "SwerveModuleState.h"
#include "Constants.h"
#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/SparkMaxPIDController.h>

class SwerveModule
{
    // private: <- removed for testing
public:
    int steerID = 11;
    int driveID = 18;
    rev::CANSparkMax *steerMotor = new rev::CANSparkMax(steerID, rev::CANSparkMax::MotorType::kBrushless);
    rev::CANSparkMax *driveMotor = new rev::CANSparkMax(driveID, rev::CANSparkMax::MotorType::kBrushless);

    rev::SparkMaxRelativeEncoder steerEnc = steerMotor->GetEncoder();
    rev::SparkMaxRelativeEncoder driveEnc = driveMotor->GetEncoder();

    // PID Controller for Steer Motor
    frc2::PIDController steerCTR{0.2, 0.0, 0.0};

    // REV Default Velocity PID values(Drive Motor)
    double kP = 6e-5, kI = 1e-6, kD = 0, kIz = 0, kFF = 0.000015, kMaxOutput = 1.0, kMinOutput = -1.0;

    // PID Controller for Drive Motor
    rev::SparkMaxPIDController m_pidController = driveMotor->GetPIDController();

    float driveVelocitySetpoint;
    float drivePositionSetpoint;
    float steerAngleSetpoint;
    bool driveModePosition = false;
    bool stopThread = false;
    const int maxRPMFreeSpeed = 5700;

    // public:
    SwerveModule(int steerMotorID, int driveMotorID); // To be implemented, unable to initialize motors here
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