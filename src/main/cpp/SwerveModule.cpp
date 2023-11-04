#include "SwerveModule.h"

SwerveModule::SwerveModule(int steerMotorID, int driveMotorID)
{
    steerID = steerMotorID;
    driveID = driveMotorID;
}

void SwerveModule::initMotors()
{
    // Resetting Motor settings, Encoders, putting it in brake mode
    steerMotor->RestoreFactoryDefaults();
    driveMotor->RestoreFactoryDefaults();
    steerEnc.SetPosition(0);
    driveEnc.SetPosition(0);
    steerMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    driveMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    // Keep the motor limit at under 20A
    steerMotor->SetSmartCurrentLimit(20);
    driveMotor->SetSmartCurrentLimit(20);

    // Conversion factor from Rotations of motor, which is nothing for now
    steerEnc.SetPositionConversionFactor(1.0);
    driveEnc.SetPositionConversionFactor(1.0);

    // Setpoints to initial encoder positions/speeds
    steerAngleSetpoint = steerEnc.GetPosition();
    driveVelocitySetpoint = 0.0;
}

float SwerveModule::getSteerAngleSetpoint()
{
    return steerAngleSetpoint;
}

void SwerveModule::setSteerAngleSetpoint(float setpt)
{
    steerAngleSetpoint = setpt;
}

/* Takes in input from 0 - 2pi
 * 0 is the right, goes counterclockwise
 *
 */
void SwerveModule::setSteerAngleSetpointShortestPath(float setpt)
{
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

void SwerveModule::setModuleState(SwerveModuleState setpt)
{
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
        if (stopThread) // Thread is in standby mode
        {

            steerMotor->StopMotor();
            driveMotor->StopMotor();
        }

        else
        {
            double driveOutput = driveCTR.Calculate(driveEnc.GetVelocity(), driveVelocitySetpoint);
            double steerOutput = steerCTR.Calculate(steerEnc.GetPosition(), steerAngleSetpoint);

            if (driveModePosition)
            {
                double driveOutput = driveCTR.Calculate(driveEnc.GetPosition(), drivePositionSetpoint);
            }
            else
            {
                double driveOutput = driveCTR.Calculate(driveEnc.GetVelocity(), driveVelocitySetpoint);
            }

            frc::SmartDashboard::PutNumber("driveOut", driveOutput);
            frc::SmartDashboard::PutNumber("steerOut", steerOutput);

            steerMotor->Set(steerOutput);
            driveMotor->Set(driveOutput);
        }
    }
}

Rotation2d SwerveModule::getSteerEncoder()
{
    return Rotation2d(steerEnc.GetPosition());
}

double SwerveModule::getDriveEncoderVel()
{
    return driveEnc.GetVelocity();
}

double SwerveModule::getDriveEncoderPos()
{
    return driveEnc.GetPosition();
}

void SwerveModule::standbyThread()
{
    stopThread = true;
}

void SwerveModule::exitStandbyThread()
{
    stopThread = false;
}