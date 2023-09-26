#include <rev/CANSparkMax.h>
#include <thread>
#include "Translation2d.h"
#include "SwerveModuleState.h"

class SwerveModule
{
private:
    int steerID;
    int driveID;
    rev::CANSparkMax *steerMotor = new rev::CANSparkMax(steerID, rev::CANSparkMax::MotorType::kBrushless);
    rev::CANSparkMax *driveMotor = new rev::CANSparkMax(driveID, rev::CANSparkMax::MotorType::kBrushless);

    rev::SparkMaxPIDController steerPID = steerMotor->GetPIDController();
    rev::SparkMaxPIDController drivePID = driveMotor->GetPIDController();

    rev::SparkMaxRelativeEncoder steerEnc = steerMotor->GetEncoder();
    rev::SparkMaxRelativeEncoder driveEnc = driveMotor->GetEncoder();

    double steerP;
    double steerI;
    double steerD;
    double driveP;
    double driveI;
    double driveD;

    float driveVelocitySetpoint;
    float drivePositionSetpoint;
    float steerAngleSetpoint;
    bool driveModePosition = false;
    bool stopThread = false;



public:
    SwerveModule(int steerMotorID, int driveMotorID);
    void initMotors();
    float getSteerAngleSetpoint();
    void setSteerAngleSetpoint(float setpt);
    void setSteerAngleSetpointShortestPath(float setpt);
    void setDrivePositionSetpoint(float setpt);
    void setDriveVelocitySetpoint(float setpt);
    void setModuleState(SwerveModuleState setpt);
    double getSteerEncoder();
    double getDriveEncoder();
    bool isFinished(float percentageBound);
    void run();
    void joinThread();



};