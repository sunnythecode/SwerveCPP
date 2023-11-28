
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
    SwerveModule mFrontLeft = SwerveModule(10, 13);
    SwerveModule mBackRight = SwerveModule(4, 16);
    SwerveModule mBackLeft = SwerveModule(1, 42);
    SwerveModule mFrontRight = SwerveModule(11, 18);


    SwerveModule mModules[4] = {mFrontLeft, mFrontRight, mBackLeft, mBackRight};

    std::thread moduleThreads[4];

    float maxSpeed = 5700;
    float maxRot = 0.5; // Rad / sec?

    float trackWidth = 1.0;
    float wheelBase = 1.0;

    std::vector<Translation2d> wheelPs = {Translation2d(trackWidth, wheelBase), Translation2d(trackWidth, -wheelBase), Translation2d(-trackWidth, wheelBase), Translation2d(-trackWidth, -wheelBase)};
    SwerveDriveKinematics m_kinematics = SwerveDriveKinematics(wheelPs);

public:
    void Drive(double rightX, double leftX, double leftY, double fieldRelativeGyro);
    void setModuleVelocity(SwerveModule &mModule, double speed, double angleRadians);
    void initAllMotors();
    void initThreads();
    bool stopAllMotors();
};
