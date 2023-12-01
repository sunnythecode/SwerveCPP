#include "SwerveModule.h"

SwerveModule::SwerveModule(int steerMotorID, int driveMotorID, int CAN_ID) : steerMotor(new rev::CANSparkMax(steerMotorID, rev::CANSparkMax::MotorType::kBrushless)),
                                                                 driveMotor(new rev::CANSparkMax(driveMotorID, rev::CANSparkMax::MotorType::kBrushless)),
                                                                 steerEnc(CAN_Coder(CAN_ID)),
                                                                 driveEnc(driveMotor->GetEncoder()),
                                                                 m_pidController(driveMotor->GetPIDController())
{
    steerID = steerMotorID;
    driveID = driveMotorID;
}

void SwerveModule::initMotors()
{
    // Resetting Motor settings, Encoders, putting it in brake mode
    steerMotor->RestoreFactoryDefaults();
    driveMotor->RestoreFactoryDefaults();
    steerMotor->SetInverted(false);
    steerEnc.encoder.SetPosition(0);
    driveEnc.SetPosition(0);
    steerMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    driveMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    // Keep the motor limit at under 20A
    steerMotor->SetSmartCurrentLimit(20);
    driveMotor->SetSmartCurrentLimit(20);

    // Conversion factor from Rotations of motor, which is nothing for now
    driveEnc.SetPositionConversionFactor(1.0);

    // Setpoints to initial encoder positions/speeds
    steerAngleSetpoint = steerEnc.getPosition().getRadians();
    driveVelocitySetpoint = 0.0;

    // Set PID values for REV Drive PID
    m_pidController.SetP(kP);
    m_pidController.SetI(kI);
    m_pidController.SetD(kD);
    m_pidController.SetIZone(kIz);
    m_pidController.SetFF(kFF);
    m_pidController.SetOutputRange(kMinOutput, kMaxOutput);

    steerCTR.EnableContinuousInput(0, 2 * M_PI);
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
    double currAngle = steerEnc.getPosition().getRadians();
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

/**
 * Set the drive motor velocity setpoint to input RPM
 *
 */
void SwerveModule::setDriveVelocitySetpoint(float setpt)
{
    driveVelocitySetpoint = setpt;
    driveModePosition = false;
}

/**
 * Set the drive motor velocity setpoint to the input percent of max RPM
 *
 */
void SwerveModule::setDrivePercentVelocitySetpoint(float setpt)
{
    setDriveVelocitySetpoint(maxRPMFreeSpeed * setpt);
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
            frc::SmartDashboard::PutNumber("SteerSetpoint", steerAngleSetpoint);
            frc::SmartDashboard::PutNumber("SteerPos", steerEnc.getPosition().getRadians());

            double steerOutput = steerCTR.Calculate(steerEnc.getPosition().getRadians(), steerAngleSetpoint);
            //frc::SmartDashboard::PutNumber("SteerOutput" + std::to_string(steerID), steerOutput);
            frc::SmartDashboard::PutNumber("SteerOutput", steerOutput);
            steerMotor->Set(steerOutput);

            // Drive Motor uses the internal REV PID, since optimizations here are rarely needed
            if (driveModePosition)
            {
                m_pidController.SetReference(drivePositionSetpoint, rev::CANSparkMax::ControlType::kPosition);
            }
            else
            {
                m_pidController.SetReference(driveVelocitySetpoint, rev::CANSparkMax::ControlType::kVelocity);
            }
        }
    }
}

Rotation2d SwerveModule::getSteerEncoder()
{
    return steerEnc.getPosition();
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