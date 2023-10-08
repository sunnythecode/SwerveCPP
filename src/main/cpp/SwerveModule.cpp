#include "SwerveModule.h"

SwerveModule::SwerveModule(int steerMotorID, int driveMotorID)
{
    steerID = steerMotorID;
    driveID = driveMotorID;
}

void SwerveModule::initMotors()
{
    steerMotor->RestoreFactoryDefaults();
    driveMotor->RestoreFactoryDefaults();
    steerEnc.SetPosition(0);
    driveEnc.SetPosition(0);
    steerMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    driveMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    steerMotor->SetSmartCurrentLimit(20);
    driveMotor->SetSmartCurrentLimit(20);

    //steerEnc.SetPositionConversionFactor(steerEncConvFactor); // Rotations to Radians
    //driveEnc.SetPositionConversionFactor(driveEncConvFactor);

    steerPID.SetP(steerP);
    steerPID.SetI(steerI);
    steerPID.SetD(steerD);

    drivePID.SetP(driveP);
    drivePID.SetI(driveI);
    drivePID.SetD(driveD);

    steerPID.SetOutputRange(-1.0, 1.0);
    drivePID.SetOutputRange(-1.0, 1.0);
}

float SwerveModule::getSteerAngleSetpoint()
{
    return steerAngleSetpoint;
}

void SwerveModule::setSteerAngleSetpoint(float setpt)
{
    steerAngleSetpoint = setpt;
    steerPID.SetReference(steerAngleSetpoint, rev::CANSparkMax::ControlType::kPosition);
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
    drivePID.SetReference(drivePositionSetpoint, rev::CANSparkMax::ControlType::kPosition);

}

void SwerveModule::setDriveVelocitySetpoint(float setpt)
{
    driveVelocitySetpoint = setpt;
    driveModePosition = false;
    drivePID.SetReference(driveVelocitySetpoint, rev::CANSparkMax::ControlType::kVelocity);
}

void SwerveModule::setModuleState(SwerveModuleState setpt) {
    setDriveVelocitySetpoint(setpt.getSpeedMPS());
    setSteerAngleSetpoint(setpt.getRot2d().getRadians());
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
        if (stopThread) //Thread is in standby mode
        {
            
            steerMotor->StopMotor();
            driveMotor->StopMotor();
            break;
        } 
        
        else //Else 
        {
            printf("%.6f", steerAngleSetpoint);
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
}

Rotation2d SwerveModule::getSteerEncoder() {
    return Rotation2d(steerEnc.GetPosition());
}

double SwerveModule::getDriveEncoder() {
    return driveEnc.GetPosition();
}

void SwerveModule::joinThread()
{
    stopThread = true;
}