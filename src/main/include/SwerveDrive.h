#pragma once

#include "SwerveModule.h"
#include "Rotation2d.h"
#include "ChassisSpeeds.h"
#include "Translation2d.h"
#include "Constants.h"
#include "SwerveDriveKinematics.h"
#include "SwerveModuleState.h"
#include <thread>
#include <frc/shuffleboard/Shuffleboard.h>
#include <networktables/NetworkTableEntry.h>

class SwerveDrive
{
private:

    frc::ShuffleboardTab& tab = frc::Shuffleboard::GetTab("Swerve");


    SwerveModule mFrontLeft = SwerveModule(FLsteerID, FLdriveID, FL_CAN_ID);
    SwerveModule mFrontRight = SwerveModule(FRsteerID, FRdriveID, FR_CAN_ID);
    SwerveModule mBackLeft = SwerveModule(BLsteerID, BLdriveID, BL_CAN_ID);
    SwerveModule mBackRight = SwerveModule(BRsteerID, BRdriveID, BR_CAN_ID);

    
    // Threas for each Module
    std::thread modulePIDThread;

    float maxSpeed = moduleMaxFPS; // feet/sec, since 5700 RPM = 16 ft/s * 356.25, we have conversion factor
    float maxRot = moduleMaxRot;

    // Kinematic module: wheelPs creates x,y coordinates for each module with 0,0 being center of the robot
    std::vector<Translation2d> wheelPs = {Translation2d(trackWidth, wheelBase), Translation2d(trackWidth, -wheelBase), Translation2d(-trackWidth, wheelBase), Translation2d(-trackWidth, -wheelBase)};
    SwerveDriveKinematics m_kinematics = SwerveDriveKinematics(wheelPs);


    // Module Level functions
    void runModules(); // Private - do not call outside of init

public:
    void Drive(double rightX, double leftX, double leftY, double fieldRelativeGyro);
    void setModuleVelocity(SwerveModule &mModule, double speed, double angleRadians);
    void initAllMotors();
    void enableThreads();
    bool stopAllMotors();
    double convertAngleReference(double input);
    void orientModules(double FL, double FR, double BL, double BR);
    void autoMove(double angleRadians, double distanceFeet);
    // void autoRotate(double angleRadians); Needs to be thought out a little
};
