
#include "SwerveDrive.h"
#include <cmath>

/*
 * Steps to basic swerve drive
 * Left stick controls robot velocity(direction + speed)
 * Right stick X controls robot rotation
 *
 * Field Relative swerve drive: Convert desired speeds to robot relative desired speeds
 *
 *
 */

void SwerveDrive::Drive(double rightX, double leftX, double leftY, double fieldRelativeGyro)
{


    if ((fabs(leftX) < 0.1) && (fabs(leftY) < 0.1)) { // No magnitude

        if (fabs(rightX) < 0.1) {
            // No movement
            mBackRight.setDrivePercentVelocitySetpoint(0.0);
            mBackLeft.setDrivePercentVelocitySetpoint(0.0);
            mFrontRight.setDrivePercentVelocitySetpoint(0.0);
            mFrontLeft.setDrivePercentVelocitySetpoint(0.0);
        } 
        else
        {
            // Rotate in Place
            //Rotational Positions
            orientModules(M_PI / 4, -M_PI / 4, -M_PI / 4, M_PI / 4);
            // No need for kinematics, just use rightX as a percent speed
            mBackRight.setDrivePercentVelocitySetpoint(rightX);
            //mBackLeft.setDrivePercentVelocitySetpoint(rightX);
            mFrontRight.setDrivePercentVelocitySetpoint(rightX);
            mFrontLeft.setDrivePercentVelocitySetpoint(rightX);

        }
    } else {
        // Creating desired Chassis speeds from controller input
        double Vx = leftX * maxSpeed;
        double Vy = leftY * maxSpeed;
        double omega = rightX * maxRot;

        ChassisSpeeds desiredSpeeds = ChassisSpeeds::fromFieldRelativeSpeeds(Vx, Vy, omega, fieldRelativeGyro);
        
        // Feeding chassis speeds into kinematics module(which works, I tested it)
        std::vector<SwerveModuleState> moduleStates = m_kinematics.toSwerveStates(desiredSpeeds);

        // Printing the setpoints for our single module
        // BTW order of motors is FL, FR, BL, BR so [2] corresponds to BL
        frc::SmartDashboard::PutNumber("Degree", moduleStates[2].getRot2d().getDegrees());
        frc::SmartDashboard::PutNumber("Speed", moduleStates[2].getSpeedMPS());
          // double mag = sqrt((ctr->GetLeftY() * ctr->GetLeftY()) + (ctr->GetLeftX() * ctr->GetLeftX()));


        // Convert angle reference to 0 is in front
        // Convert ft/s to motor RPM(x356.25)
        for (int i = 0; i < 4; i++) {
            SwerveModuleState temp = SwerveModuleState(moduleStates[i].getSpeedMPS() * 356.25, convertAngleReference(moduleStates[i].getRot2d().getRadians()));
            moduleStates[i] = temp;
        }

        mFrontLeft.setModuleState(moduleStates[0]);
        mFrontRight.setModuleState(moduleStates[1]);
        mBackRight.setModuleState(moduleStates[2]);
        //mBackLeft.setModuleState(moduleStates[3]);

    }


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

double SwerveDrive::convertAngleReference(double angle) {
    angle = -angle + M_PI_2;
    angle = angle * 180 / M_PI;
    angle = fmod(angle + 360, 360);
    angle = angle * M_PI / 180;

    return angle;
    
}
/**
 * Enter radians
*/
void SwerveDrive::orientModules(double FL, double FR, double BL, double BR) {
    mBackRight.setSteerAngleSetpoint(BR);
    mBackLeft.setSteerAngleSetpoint(BL);
    mFrontRight.setSteerAngleSetpoint(FR);
    mFrontLeft.setSteerAngleSetpoint(FL);

}