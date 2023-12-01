#pragma once

#include "SwerveModule.h"
#include "Rotation2d.h"
#include "ChassisSpeeds.h"
#include "Translation2d.h"
#include "SwerveDriveKinematics.h"
#include "SwerveModuleState.h"
#include <thread>

class SwerveDrive
{
private:
    SwerveModule mFrontLeft = SwerveModule(11, 18, 3);
    SwerveModule mBackRight = SwerveModule(1, 42, 0);
    SwerveModule mBackLeft = SwerveModule(3, 10, 2); 
    SwerveModule mFrontRight = SwerveModule(4, 16, 1);

    std::thread FLthread;
    std::thread FRthread;
    std::thread BLthread;
    std::thread BRthread;



    //SwerveModule mModules[4] = {mFrontLeft, mFrontRight, mBackLeft, mBackRight};

    // Declaring threads, initializing them in initAllMotors()
    

    float maxSpeed = 5700; // This is the NEO max RPM, eventually need to convert this
    float maxRot = 1.0; // Rad / sec? - not working with rn

    float trackWidth = 2.375; // feet
    float wheelBase = 2.375; // feet

    std::vector<Translation2d> wheelPs = {Translation2d(trackWidth, wheelBase), Translation2d(trackWidth, -wheelBase), Translation2d(-trackWidth, wheelBase), Translation2d(-trackWidth, -wheelBase)};
    SwerveDriveKinematics m_kinematics = SwerveDriveKinematics(wheelPs);

public:
    void Drive(double rightX, double leftX, double leftY, double fieldRelativeGyro);
    void setModuleVelocity(SwerveModule &mModule, double speed, double angleRadians);
    void initAllMotors();
    void enableThreads();
    bool stopAllMotors();
    double convertAngleReference(double input);
};
