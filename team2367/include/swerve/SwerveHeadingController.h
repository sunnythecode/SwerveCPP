#pragma once
#include <Constants.h>
#include "geometry/Pose2d.h"
#include <cmath>
#include <stdexcept>
#include <frc/controller/PIDController.h>


class SwerveHeadingController {
private:
    frc2::PIDController mPIDCtr {Constants::kSnapSwerveHeadingKp, Constants::kSnapSwerveHeadingKi, Constants::kSnapSwerveHeadingKd};
    double mSetpoint = 0.0;
        
    
    Rotation2d desiredHeading;

public:
    enum HeadingControllerState {
        OFF, SNAP
    };
    HeadingControllerState mHeadingControllerState = OFF;


    SwerveHeadingController() {
        mPIDCtr.EnableContinuousInput(0, 360);
    }

    HeadingControllerState getHeadingControllerState() {
        return mHeadingControllerState;
    }

    void setHeadingControllerState(HeadingControllerState state) {
        mHeadingControllerState = state;
    }

    double calculate(double current_angle, double setpoint) {
        mSetpoint = setpoint;
        double current_error = mSetpoint - current_angle;

        switch (mHeadingControllerState) {
            case OFF:
                return 0.0;
            case SNAP:
                mPIDCtr.SetPID(Constants::kSnapSwerveHeadingKp, Constants::kSnapSwerveHeadingKi, Constants::kSnapSwerveHeadingKd);
                break;
        }

        return mPIDCtr.Calculate(current_angle, mSetpoint);
    }


};
