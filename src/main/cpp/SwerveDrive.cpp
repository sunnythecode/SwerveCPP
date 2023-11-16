
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
    

    // double Vx = leftX * maxSpeed;
    // double Vy = leftY * maxSpeed;
    // double omega = rightX * maxRot;

    // Simple Directional Swerve - Assume are modules facing forward
    double vel_mag = sqrt((leftX * leftX) + (leftY + leftY)) / sqrt(2);
    double angle = atan2(leftY, leftX);

    SwerveModuleState setpts(vel_mag, angle);

    mFrontLeft.setModuleState(setpts);
    mFrontRight.setModuleState(setpts);
    mBackRight.setModuleState(setpts);
    mBackLeft.setModuleState(setpts);


    
    






    // ChassisSpeeds desiredSpeeds = ChassisSpeeds::fromFieldRelativeSpeeds(Vx, Vy, omega, fieldRelativeGyro);

    // std::vector<SwerveModuleState> moduleStates = m_kinematics.toSwerveStates(desiredSpeeds);
    
    // mFrontLeft.setModuleState(moduleStates[0]);
    // mFrontRight.setModuleState(moduleStates[1]);
    // mBackRight.setModuleState(moduleStates[2]);
    // mBackLeft.setModuleState(moduleStates[3]);

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
        moduleThreads[i] = std::thread(&SwerveModule::run, &mModules[i]);
    }
}

void SwerveDrive::enableMotors() 
{
    for (int i = 0; i < 4; i++) 
    {
        mModules[i].exitStandbyThread();
    }

}

bool SwerveDrive::disableMotors()
{
    for (int i = 0; i < 4; i++)
    {
        mModules[i].standbyThread();
    }
    return true;
}
