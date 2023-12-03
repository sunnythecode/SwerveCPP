#pragma once

#include "Constants.h"
#include <cmath>

class Rotation2d
{
private:
    double radians;


public:

    Rotation2d() {
        radians = 0.0;

    }
    
    Rotation2d(double angleRadians)
    {
        radians = angleRadians;
    }

    // Returns a new Rotation2d object that is the result of adding two rotations.
    Rotation2d operator+(const Rotation2d &other) const
    {
        double sumRadians = radians + other.radians;
        return Rotation2d(sumRadians);
    }

    // Returns a new Rotation2d object that is the inverse of the current rotation.
    Rotation2d inverse() const
    {
        double inverseRadians = -radians;
        return Rotation2d(inverseRadians);
    }

    // Converts the rotation to degrees.
    double getDegrees() const
    {
        return radians * 180.0 / PI;
    }

    double getRadians() const
    {
        return radians;
    }

    static double radiansBound(double input_radians) {
        double deg = (180 / M_PI) * input_radians;
        double output = fmod(fmod(deg, 360) + 360, 360) * (M_PI / 180);
        return output;

    }

    static double degreesBound(double input_degrees) {
        double deg = input_degrees;
        double output = fmod(fmod(deg, 360) + 360, 360);
        return output;
    }


};
