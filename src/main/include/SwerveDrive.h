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
    SwerveModule mFrontLeft = SwerveModule(0, 1);
    SwerveModule mFrontRight = SwerveModule(2, 3);
    SwerveModule mBackLeft = SwerveModule(4, 5);
    SwerveModule mBackRight = SwerveModule(6, 7);
    SwerveModule mModules[4] = {mFrontLeft, mFrontRight, mBackLeft, mBackRight};

    std::thread moduleThreads[4] = {std::thread(&SwerveModule::run, &mFrontLeft),
                                    std::thread(&SwerveModule::run, &mFrontRight),
                                    std::thread(&SwerveModule::run, &mBackLeft),
                                    std::thread(&SwerveModule::run, &mBackRight)};

    float maxSpeed = 1.5;
    float maxRot = 1; // Rad / sec?

    float trackWidth = 1.0;
    float wheelBase = 1.0;

    std::vector<Translation2d> wheelPs = {Translation2d(trackWidth, wheelBase), Translation2d(trackWidth, -wheelBase), Translation2d(-trackWidth, wheelBase), Translation2d(-trackWidth, -wheelBase)};
    SwerveDriveKinematics m_kinematics = SwerveDriveKinematics(wheelPs);

public:
    void Drive(double rightX, double leftX, double leftY, double fieldRelativeGyro);
    void setModuleVelocity(SwerveModule &mModule, double speed, double angleRadians);
    void initAllMotors();
    bool stopAllMotors();
};