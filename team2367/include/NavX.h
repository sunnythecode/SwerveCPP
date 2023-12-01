#pragma once

#include "AHRS.h"


class NavX {
    public:


    AHRS gyro = AHRS(frc::SerialPort::kMXP);


    void init() {
        gyro.Reset();
    }

    




};