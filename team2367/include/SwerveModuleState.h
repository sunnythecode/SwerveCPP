#pragma once

#include "Rotation2d.h"

class SwerveModuleState {
    private:
        double speed_;
        double angleRadians_;

    public:
        SwerveModuleState(double speedMPS, double angleRadians) {
            speed_ = speedMPS;
            angleRadians_ = angleRadians;
        }
        SwerveModuleState(double speedMPS, double angleDegrees) {
            speed_ = speedMPS;
            angleRadians_ = angleDegrees * M_PI / 180.0;
        }

        SwerveModuleState(double speedMPS, Rotation2d rotRadians) {
            speed_ = speedMPS;
            angleRadians_ = rotRadians.getRadians();
        }





};