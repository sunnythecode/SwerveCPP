#include "SwerveDrive.h"
#include <cmath>

/*
 * Steps to basic swerve drive
 * Left stick controls robot velocity(direction + speed)
 * Right stick controls robot rotation
 *
 *
 *
 *
 */
void SwerveDrive::Drive(double rightX, double leftX, double leftY)
{
    /* Idea here is that controller input is a circle
     * Normalizing the x, y to maxSpeed will not create a hyp > maxSpeed
     * At pi/4 rad, the x, y both are root(2)
     */

    double Vx = leftX * maxSpeed;
    double Vy = leftY * maxSpeed;
    double omega = rightX * maxRot;

    ChassisSpeeds desiredSpeeds = ChassisSpeeds(Vx, Vy, omega);

    desiredSpeedToModuleStates(desiredSpeeds);
}

void SwerveDrive::desiredSpeedToModuleStates(ChassisSpeeds desired)
{
    double A = desired.vxMetersPerSecond - (desired.omegaRadiansPerSecond * wheelBase / 2);
    double B = desired.vxMetersPerSecond + (desired.omegaRadiansPerSecond * wheelBase / 2);
    double C = desired.vyMetersPerSecond - (desired.omegaRadiansPerSecond * trackWidth / 2);
    double D = desired.vyMetersPerSecond + (desired.omegaRadiansPerSecond * trackWidth / 2);

    setModuleVelocity(mFrontLeft, sqrt((B * B) + (D * D)), atan2(B, D));
    setModuleVelocity(mFrontRight, sqrt((B * B) + (C * C)), atan2(B, C));
    setModuleVelocity(mBackLeft, sqrt((A * A) + (D * D)), atan2(A, D));
    setModuleVelocity(mBackRight, sqrt((A * A) + (C * C)), atan2(A, C));
}

void SwerveDrive::setModuleVelocity(SwerveModule &mModule, double speed, double angleRadians)
{
    mModule.setDriveVelocitySetpoint(speed);
    mModule.setSteerAngleSetpoint(angleRadians);
}

void SwerveDrive::initAllMotors()
{
    for (int i = 0; i < 4; i++)
    {
        mModules[i].initMotors();
        moduleThreads[i].detach();
    }
}

bool SwerveDrive::stopAllMotors()
{
    for (int i = 0; i < 4; i++)
    {
        mModules[i].joinThread();
        moduleThreads[i].join();
    }
    return true;
}
