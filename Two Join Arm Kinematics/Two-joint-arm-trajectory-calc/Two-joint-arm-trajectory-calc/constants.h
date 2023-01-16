#pragma once

#include <math.h>
#include "string"

using namespace std;

namespace GeneralConstants
{
    const double Kdt = 0.005; //0.005, 0.02
    const int MAX_RPM = 6380;
    const int TICKS_PER_ROTATION = 2048;

    const double FREE_SPEED = 6380;
    const double FREE_CURRENT = 1.5;
    const double STALL_CURRENT = 257;
    const double MAX_VOLTAGE = 12;

    const double RESISTANCE = MAX_VOLTAGE / STALL_CURRENT;
    const double Kv = ((FREE_SPEED * 2 * 3.1415) / 60) / (MAX_VOLTAGE - FREE_CURRENT * RESISTANCE);

    const double GOAL_HEIGHT = 2.641;
    const double GOAL_RADIUS = 0.6096;

    const double HANGAR_X = -2.5;
    const double HANGAR_Y = -6;
    const double FIELD_WIDTH = 8.2296;
    const double FIELD_LENGTH = 16.4592;
    const double HUB_BASE_RADIUS = 0.5;

    const int MAX_BALL_COUNT = 1;

}

namespace TwoJointArmConstants
{
	const double LOWER_ARM_LENGTH = 0.635;
	const double UPPER_ARM_LENGTH = 0.8636;
	
	const double LOWER_ARM_MIN_ANG = -170;
	const double LOWER_ARM_MAX_ANG = 170;
	const double UPPER_ARM_MIN_ANG = -170;
	const double UPPER_ARM_MAX_ANG = 170;

	const double LOWER_ARM_MAX_VEL = 90;
	const double UPPER_ARM_MAX_VEL = 90;
	const double LOWER_ARM_MAX_ACC = 45;
	const double UPPER_ARM_MAX_ACC = 45;
}