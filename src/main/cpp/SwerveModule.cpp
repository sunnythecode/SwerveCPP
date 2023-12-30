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

    // No inverts needed due to CANCoder
    steerMotor->SetInverted(false);

    // To be changed to absolute position
    steerEnc.encoder.SetPosition(steerEnc.getAbsolutePositionDeg().getDegrees());
    driveEnc.SetPosition(0);

    // Makes motor stiff(coast mode lets it run freely)
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

/**
 * Enter in radians
 * Will modulus it to 0 - 2pi
 */
void SwerveModule::setSteerAngleSetpoint(float setpt)
{
    steerAngleSetpoint = Rotation2d::radiansBound(setpt);
}

/* Takes in input from 0 - 2pi
 * 0 is the right, goes counterclockwise
 * Not tested
 */
bool SwerveModule::setSteerAngleSetpointShortestPath(float setpt)
{
    double currAngle = Rotation2d::radiansBound(steerEnc.getPosition().getRadians());
    double setAngle = Rotation2d::radiansBound(setpt);
    bool flip = false;
    if (fabs(currAngle - setAngle) > (M_PI / 2)) // Flipping drive direction = shorter
    {
        flip = true;
        setSteerAngleSetpoint(setAngle - (M_PI / 2));
    } else 
    {
        setSteerAngleSetpoint(setAngle);
        
    }
    return flip;
}

/**
 * Untested, I've never used it
 */
void SwerveModule::setDrivePositionSetpoint(float setpt)
{
    drivePositionSetpoint = setpt;
    driveModePosition = true;
}

/**
 * Set the drive motor velocity setpoint to input RPM
 * Max RPM is 5700
 */
void SwerveModule::setDriveVelocitySetpoint(float setpt)
{
    driveVelocitySetpoint = setpt;
    driveModePosition = false;
}

/**
 * Set the drive motor velocity setpoint to the input percent of max RPM
 *  Input shld be in [-1, 1]
 */
void SwerveModule::setDrivePercentVelocitySetpoint(float setpt)
{
    setDriveVelocitySetpoint(maxRPMFreeSpeed * setpt);
}

/**
 * speedMPS attribute should be in RPM
 * Sets Drive Velocity & Steer Angle
 */
void SwerveModule::setModuleState(SwerveModuleState setpt)
{
    bool flip = setSteerAngleSetpointShortestPath(setpt.getRot2d().getRadians());
    if (flip) {
        setDriveVelocitySetpoint(-setpt.getSpeedMPS());
    } else {
        setDriveVelocitySetpoint(setpt.getSpeedMPS());
    }
}

/**
 * Unfinished, Untested, pending review
 */
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

/**
 * This function is meant to run in a while loop
 * when stopThread is true, motors are stopped
 * when stopThread is false, motor PIDs are running
 * steerPID uses frc2::PIDController
 * drivePID uses rev::PIDController
 *
 */
void SwerveModule::run()
{

    if (stopThread) // Thread is in standby mode
    {

        steerMotor->StopMotor();
        driveMotor->StopMotor();
    }

    else
    {
        // Steer PID
        double steerOutput = steerCTR.Calculate(steerEnc.getAbsolutePositionDeg().getRadians(), steerAngleSetpoint);
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

Rotation2d SwerveModule::getSteerEncoder()
{
    return steerEnc.getAbsolutePositionDeg();
}

double SwerveModule::getDriveEncoderVel()
{
    return driveEnc.GetVelocity();
}

double SwerveModule::getDriveEncoderPos()
{
    return driveEnc.GetPosition();
}

/**
 * Set stopThread to true
 * Stops motors and exits PID loop
 * Intended for disabledInit()
 *
 */
void SwerveModule::standbyThread()
{
    stopThread = true;
}

/**
 * Set stopThread to false
 * Enter PID loop, motors are ON
 * Intended for teleop/auto init functions
 */
void SwerveModule::exitStandbyThread()
{
    stopThread = false;
}