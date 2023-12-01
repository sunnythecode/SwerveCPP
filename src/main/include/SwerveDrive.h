// #pragma once

// #include "SwerveModule.h"
// #include "Rotation2d.h"
// #include "ChassisSpeeds.h"
// #include "Translation2d.h"
// #include "SwerveDriveKinematics.h"
// #include "SwerveModuleState.h"
// #include <thread>

// class SwerveDrive
// {
// private:
//     SwerveModule mFrontLeft = SwerveModule(10, 13);
//     SwerveModule mBackRight = SwerveModule(4, 16);
//     SwerveModule mBackLeft = SwerveModule(1, 42);
//     SwerveModule mFrontRight = SwerveModule(11, 18);


//     SwerveModule mModules[4] = {mFrontLeft, mFrontRight, mBackLeft, mBackRight};

//     // Declaring threads, initializing them in initAllMotors()
//     std::thread moduleThreads[4];

//     float maxSpeed = 5700; // This is the NEO max RPM, eventually need to convert this
//     float maxRot = 0.5; // Rad / sec? - not working with rn

//     float trackWidth = 2.375; // feet
//     float wheelBase = 2.375; // feet

//     std::vector<Translation2d> wheelPs = {Translation2d(trackWidth, wheelBase), Translation2d(trackWidth, -wheelBase), Translation2d(-trackWidth, wheelBase), Translation2d(-trackWidth, -wheelBase)};
//     SwerveDriveKinematics m_kinematics = SwerveDriveKinematics(wheelPs);

// public:
//     void Drive(double rightX, double leftX, double leftY, double fieldRelativeGyro);
//     void setModuleVelocity(SwerveModule &mModule, double speed, double angleRadians);
//     void initAllMotors();
//     void enableThreads();
//     bool stopAllMotors();
// };
