#pragma once
#include <Constants.h>
#include "geometry/Pose2d.h"
#include <cmath>
#include <stdexcept>
#include <util/SynchronousPIDF.h>
class SwerveHeadingController {
public:
    Pose2d centerOfGoal;
    enum HeadingControllerState {
        OFF, SNAP, MAINTAIN, POLAR_MAINTAIN, POLAR_SNAP
    };
    HeadingControllerState mHeadingControllerState = OFF;
    static SwerveHeadingController& getInstance() {
        static SwerveHeadingController instance;
        return instance;
    }

    HeadingControllerState getHeadingControllerState() {
        return mHeadingControllerState;
    }

    void setHeadingControllerState(HeadingControllerState state) {
        mHeadingControllerState = state;
    }

    void setGoal(double goal_pos) {
        mSetpoint = goal_pos;
    }

    double getGoal() {
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
       mPIDFController.setSetpoint(mSetpoint);
        double current_error = mSetpoint - current_angle;

        if (current_error > 180) {
            current_angle += 360;
        } else if (current_error < -180) {
            current_angle -= 360;
        }
        switch (mHeadingControllerState) {
            case OFF:
                return 0.0;
            case SNAP:
                mPIDFController.setPID(0.05, 0.0, 0.075);
                break;
            // case MAINTAIN:
            //     mPIDFController.setPID(kMaintainSwerveHeadingKpHighVelocity, 0, 0);
            //     mPIDFController.setOutputRange(-1.0, 1.0);
            //     break;
            // case POLAR_MAINTAIN:
            //     //mPIDFController.setPID(Constants.kMaintainSwerveHeadingKp, Constants.kMaintainSwerveHeadingKi, Constants.kMaintainSwerveHeadingKd);
            //     break;
            // case POLAR_SNAP:
            //     mPIDFController.setPID(kSnapSwerveHeadingKp, kSnapSwerveHeadingKi, kSnapSwerveHeadingKd);
            //     break;
        }

        return mPIDFController.calculate(current_angle);
    }
private:
    static SwerveHeadingController mInstance;
    SynchronousPIDF mPIDFController;
    double mSetpoint = 0.0;
    // Define constants like kSwerveHeadingControllerErrorTolerance and others here
    // You'll need to replace these constants with their C++ equivalents

    // SwerveHeadingController() = default;
    // SwerveHeadingController(const SwerveHeadingController&) = delete;
    // SwerveHeadingController& operator=(const SwerveHeadingController&) = delete;
};
