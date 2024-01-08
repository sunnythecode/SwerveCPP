#pragma once
#include <Constants.h>
#include "geometry/Pose2d.h"
#include <cmath>
#include <stdexcept>
#include <util/SynchronousPIDF.h>
#include <frc/controller/PIDController.h>
class SwerveHeadingController {
public:
    Pose2d centerOfGoal;
    enum HeadingControllerState {
        OFF, SNAP
    };

    SwerveHeadingController() {
        mPIDCtr.EnableContinuousInput(0, 360);
        mPIDCtr.SetPID(0.05, 0.0, 0.075);

    }
    HeadingControllerState mHeadingControllerState = OFF;

    HeadingControllerState getHeadingControllerState() {
        return mHeadingControllerState;
    }

    void setHeadingControllerState(HeadingControllerState state) {
        mHeadingControllerState = state;
    }

    void setSetpoint(double setpt) {
        mSetpoint = setpt;
    }

    double getSetpoint() {
        return mSetpoint;
    }

    // double calculateAngleToOrigin(Pose2d current_pose) {
    //     centerOfGoal = mRobotState.getFieldToGoal();
    //     double r = current_pose.getTranslation().distance(Translation2d.identity());
    //     double theta = Math.atan2(current_pose.getTranslation().y(), current_pose.getTranslation().x());
    //     double r_central = centerOfGoal.getTranslation().distance(Translation2d.identity());
    //     double theta_central = Math.atan2(centerOfGoal.getTranslation().y(), centerOfGoal.getTranslation().x());

    //     double angle = Math.toDegrees(Math.PI + Math.atan2(r * Math.sin(theta) - r_central * Math.sin(theta_central),
    //             r * Math.cos(theta) - r_central * Math.cos(theta_central)));
    //     if(angle < 0) {
    //         return -angle;
    //     }
    //     return angle;
    // }

    double update(double current_angle) {
       // mPIDFController.setSetpoint(mSetpoint);
       double current_error = mSetpoint - current_angle;

        // if (current_error > 180) {
        //     current_angle += 360;
        // } else if (current_error < -180) {
        //     current_angle -= 360;
        // }
        switch (mHeadingControllerState) {
            case OFF:
                return 0.0;
            case SNAP:
                // mPIDFController.setPID(0.05, 0.0, 0.075);
                mPIDCtr.SetPID(0.05, 0.0, 0.075);
                break;
        }
        return mPIDCtr.Calculate(current_angle, mSetpoint);
        // return mPIDFController.calculate(current_angle);
    }
private:
    SynchronousPIDF mPIDFController;
    frc2::PIDController mPIDCtr {0.05, 0.0, 0.0};

    double mSetpoint = 0.0;
    // Define constants like kSwerveHeadingControllerErrorTolerance and others here
    // You'll need to replace these constants with their C++ equivalents

    // SwerveHeadingController() = default;
    // SwerveHeadingController(const SwerveHeadingController&) = delete;
    // SwerveHeadingController& operator=(const SwerveHeadingController&) = delete;
};
