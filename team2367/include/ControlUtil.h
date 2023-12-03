#pragma once

#include <cmath>


class ControlUtil 
{
    public:
    
    /**
     * Add deadzone and scale from deadzone by square
     * Input must be in [-1, 1]
    */
    double deadZoneScale(double input, double deadzone)
    {
        if (fabs(input) < deadzone) {
            return std::copysign(pow((fabs(input) - deadzone), 2), input);
        } else {
            return input;
        }

    }





    



};