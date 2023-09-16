#include "SwerveDrive.h"
#include <cmath>

/*
 * Steps to basic swerve drive
 * Left stick controls robot velocity(direction + speed)
 * Right stick controls robot rotation
 *
 * Field Relative swerve drive: Convert desired speeds to robot relative desired speeds
 *
 *
 */
void SwerveDrive::Drive(double rightX, double leftX, double leftY, double fieldRelativeGyro)
{
    /* Idea here is that controller input is a circle
     * Normalizing the x, y to maxSpeed will not create a hyp > maxSpeed
     * At pi/4 rad, the x, y both are root(2)
     */

    //Convert field relative desired velocities to robot relative desired velocities
    // AKA fromFieldRelativeSpeeds
    

    double Vx = leftX * maxSpeed;
    double Vy = leftY * maxSpeed;
    double omega = rightX * maxRot;

    ChassisSpeeds desiredSpeeds = ChassisSpeeds::fromFieldRelativeSpeeds(Vx, Vy, omega, fieldRelativeGyro);

    desiredSpeedToModuleStates(desiredSpeeds);
}

void SwerveDrive::desiredSpeedToModuleStates(ChassisSpeeds desired)
{
    //Ignore scenario where the center of rotation changes, because we do not intend on rotating around another point during teleop
    
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
