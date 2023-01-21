#pragma once
#include "constants.h"
#include <tuple>

class armKinematics
{
public:
	armKinematics();

	static pair<double, double> xyToAng(double x, double y, bool phiPositive);
	static pair<double, double> angToXY(double theta, double phi);

	static pair<double, double> linVelToAngVel(double xVel, double yVel, double theta, double phi);
	static pair<double, double> angVelToLinVel(double thetaVel, double phiVel, double theta, double phi);
private:
};

