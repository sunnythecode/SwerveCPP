#include "SwerveModule.h"

SwerveModule::SwerveModule(int steerMotorID, int driveMotorID)
{
    steerMotor = new rev::CANSparkMax(steerMotorID, rev::CANSparkMax::MotorType::kBrushless);
    driveMotor = new rev::CANSparkMax(driveMotorID, rev::CANSparkMax::MotorType::kBrushless);
    steerID = steerMotorID;
    driveID = driveMotorID;
}

void SwerveModule::initMotors()
{
    steerMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    driveMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    steerMotor->SetSmartCurrentLimit(20);
    driveMotor->SetSmartCurrentLimit(20);

    steerEnc.SetPositionConversionFactor(2 * M_PI); // Rotations to Radians

    // steerPID.SetP(steerP);
    // steerPID.SetI(steerI);
    // steerPID.SetD(steerD);

    // drivePID.SetP(driveP);
    // drivePID.SetI(driveI);
    // drivePID.SetD(driveD);
}

float SwerveModule::getSteerAngleSetpoint()
{
    return steerAngleSetpoint;
}

void SwerveModule::setSteerAngleSetpoint(float setpt)
{
    steerAngleSetpoint = setpt;
}

void SwerveModule::setSteerAngleSetpointShortestPath(float setpt)
{ // Input must be from 0 to 2pi
    double currAngle = steerEnc.GetPosition();
    if (fabs(setpt - currAngle) > ((2 * M_PI) - fabs(setpt - currAngle)))
    {
        steerAngleSetpoint = setpt - (2 * M_PI);
    }
    else
    {
        steerAngleSetpoint = setpt;
    }
}

void SwerveModule::setDrivePositionSetpoint(float setpt)
{
    drivePositionSetpoint = setpt;
    driveModePosition = true;
}

void SwerveModule::setDriveVelocitySetpoint(float setpt)
{
    driveVelocitySetpoint = setpt;
    driveModePosition = false;
}

bool SwerveModule::isFinished(float percentageBound)
{
    if (driveModePosition)
    {
        double pos = driveEnc.GetPosition();
        return (pos < (drivePositionSetpoint * (1 + percentageBound))) && (pos > (drivePositionSetpoint * (1 - percentageBound)));
    }
    else
    {
        double pos = driveEnc.GetVelocity();
        return (pos < (driveVelocitySetpoint * (1 + percentageBound))) && (pos > (driveVelocitySetpoint * (1 - percentageBound)));
    }
}

void SwerveModule::run()
{
    while (1)
    {
        if (stopThread)
        {
            steerMotor->StopMotor();
            driveMotor->StopMotor();
            break;
        }
        steerPID.SetReference(steerAngleSetpoint, rev::CANSparkMax::ControlType::kPosition);

        if (driveModePosition)
        {
            drivePID.SetReference(drivePositionSetpoint, rev::CANSparkMax::ControlType::kPosition);
        }
        else
        {
            drivePID.SetReference(driveVelocitySetpoint, rev::CANSparkMax::ControlType::kPosition);
        }
    }
}

void SwerveModule::joinThread()
{
    stopThread = true;
}