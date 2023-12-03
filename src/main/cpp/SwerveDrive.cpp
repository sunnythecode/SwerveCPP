
#include "SwerveDrive.h"
#include <cmath>

/*
 * Left Stick controls robot velocity(direction & speed)
 * Right Stick X controls robot rotational speed
 * Gyro is used to make the robot drive field-centric
 */

void SwerveDrive::Drive(double rightX, double leftX, double leftY, double fieldRelativeGyro)
{

    if ((fabs(leftX) < 0.1) && (fabs(leftY) < 0.1))
    { // No magnitude

        if (fabs(rightX) < 0.1)
        {
            // No movement
            mBackRight.setDrivePercentVelocitySetpoint(0.0);
            mBackLeft.setDrivePercentVelocitySetpoint(0.0);
            mFrontRight.setDrivePercentVelocitySetpoint(0.0);
            mFrontLeft.setDrivePercentVelocitySetpoint(0.0);
        }
        else
        {
            // Rotate in Place
            // Rotational Positions
            orientModules(M_PI / 4, -M_PI / 4, -M_PI / 4, M_PI / 4);
            // No need for kinematics, just use rightX as a percent speed
            mBackRight.setDrivePercentVelocitySetpoint(rightX);
            mBackLeft.setDrivePercentVelocitySetpoint(-rightX);
            mFrontRight.setDrivePercentVelocitySetpoint(rightX);
            mFrontLeft.setDrivePercentVelocitySetpoint(-rightX);
        }
    }
    else
    {
        // Creating desired Chassis speeds from controller input
        double Vx = leftX * maxSpeed;
        double Vy = leftY * maxSpeed;
        double omega = rightX * maxRot;

        //ChassisSpeeds desiredSpeeds = ChassisSpeeds::fromFieldRelativeSpeeds(Vx, Vy, omega, fieldRelativeGyro);
        ChassisSpeeds desiredSpeeds = ChassisSpeeds(Vx, Vy, omega);

        // Feeding chassis speeds into kinematics module(which works, I tested it)
        std::vector<SwerveModuleState> moduleStates = m_kinematics.toSwerveStates(desiredSpeeds);

        // Printing the setpoints for our single module
        // BTW order of motors is FL, FR, BL, BR so [2] corresponds to BL

        /**
         * Kinematics class returns module orientations in polar degrees
         * This means that 0 degrees is "to the right"
         * We want 0 degrees to be forward, so we use convertAngleReference()
         * Kinematics class also returns module speeds in ft/sec
         * We need to convert back to RPM for the PIDs, so we use our conversion factor: 356.25
         */
        for (int i = 0; i < 4; i++)
        {
            SwerveModuleState temp = SwerveModuleState(moduleStates[i].getSpeedMPS() * 356.25, convertAngleReference(moduleStates[i].getRot2d().getRadians()));
            moduleStates[i] = temp;
        }

        // Order of kinematics output is always FL, FR, BL, BR
        mFrontLeft.setModuleState(moduleStates[0]);
        mFrontRight.setModuleState(moduleStates[1]);
        mBackLeft.setModuleState(moduleStates[2]);
        mBackRight.setModuleState(moduleStates[3]);

    }


    FLentry->SetDouble(mFrontLeft.getSteerEncoder().getDegrees());
    FRentry->SetDouble(mFrontRight.getSteerEncoder().getDegrees());
    BLentry->SetDouble(mBackLeft.getSteerEncoder().getDegrees());
    BRentry->SetDouble(mBackRight.getSteerEncoder().getDegrees());
}

void SwerveDrive::setModuleVelocity(SwerveModule &mModule, double speed, double angleRadians)
{
    mModule.setDriveVelocitySetpoint(speed);
    mModule.setSteerAngleSetpoint(angleRadians);
}

/**
 * Initialize every motor(encoders, factory reset, current limits, etc)
 * Initialize each motor thread, which should start the threads
 */
void SwerveDrive::initAllMotors()
{
    mFrontLeft.initMotors();
    FLthread = std::thread(&SwerveModule::run, &mFrontLeft);
    mFrontRight.initMotors();
    FRthread = std::thread(&SwerveModule::run, &mFrontRight);
    mBackLeft.initMotors();
    BLthread = std::thread(&SwerveModule::run, &mBackLeft);
    mBackRight.initMotors();
    BRthread = std::thread(&SwerveModule::run, &mBackRight);
}
/**
 * Set every module's threads to active mode
 * So the PIDs start running
 */
void SwerveDrive::enableThreads()
{
    mFrontLeft.exitStandbyThread();
    mBackLeft.exitStandbyThread();
    mBackRight.exitStandbyThread();
    mFrontRight.exitStandbyThread();
}
/**
 * Disable every module's thread
 * Threads still exist, just on standby while loop
 */
bool SwerveDrive::stopAllMotors()
{
    mFrontLeft.standbyThread();
    mBackLeft.standbyThread();
    mBackRight.standbyThread();
    mFrontRight.standbyThread();
    return true;
}

/**
 * Enter radians
 * Converts from zero = right to zero = forward
 * Also inverts the angle
 */
double SwerveDrive::convertAngleReference(double angle)
{
    angle = -angle + M_PI_2;
    angle = angle * 180 / M_PI;
    angle = fmod(angle + 360, 360);
    angle = angle * M_PI / 180;

    return angle;
}
/**
 * Enter radians
 * Sets steer Angle setpoint to inputs
 */
void SwerveDrive::orientModules(double FL, double FR, double BL, double BR)
{
    mBackRight.setSteerAngleSetpoint(BR);
    mBackLeft.setSteerAngleSetpoint(BL);
    mFrontRight.setSteerAngleSetpoint(FR);
    mFrontLeft.setSteerAngleSetpoint(FL);
}