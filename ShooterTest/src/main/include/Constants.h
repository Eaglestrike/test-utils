#pragma once
#include <units/voltage.h>
#include <units/units.h>

namespace GeneralConstants {
    const double MAX_VOLTAGE = 12;
}

namespace ShooterConstants{
    const double HOOD_NEG_FF = 19370;
    const double HOOD_NEG_FF_INTERCEPT = 13400; 
    const double HOOD_POS_FF = 14777.1;
    const double HOOD_POS_FF_INTERCEPT = -604.762;
    const double HOOD_WEIGHT_FF = -0.69;
    const int MAX_HOOD_TICKS = -4000;

    const double MAX_HOOD_ANGLE = 60; //TODO: VERIFY
    const double TICKS_PER_HOOD_DEGREE = 166.67; //TODO: VERIFY
}

namespace SysidConstants {
    //sysid gains
    //TODO: replace 0.0 with actual gains
    const auto ks = 0.0_V;
    const auto kv = 0.0 * 1_V * 1_s / 1_rad;
    const auto ka = 0.0 * 1_V * 1_s * 1_s / 1_rad;
    const double kp = 0.0;
    const double ki = 0.0;
    const double kd = 0.0;

    const auto maxVel = 0.0_rad / 1_s;
    const auto maxAcc = 0.0_rad / (1_s * 1_s);
}