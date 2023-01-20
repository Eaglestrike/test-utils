#pragma once
#include "constants.h"
#include <tuple>

class armKinematics
{
public:
	armKinematics();

	static std::pair<double, double> xyToAng(double x, double y, bool phiPositive);
	static std::pair<double, double> angToXY(double theta, double phi);

	static std::pair<double, double> linVelToAngVel(double xVel, double yVel, double theta, double phi);
	static std::pair<double, double> angVelToLinVel(double thetaVel, double phiVel, double theta, double phi);

	static std::pair<double, double> getCenterOfMass(double theta, double phi);
	static std::pair<double, double> getTorque(double theta, double phi, double thetaAccel, double phiAccel);

private:
	// first pivot: proximal pivot
	// second pivot: distal pivot (between two segments)
	static const double l1 = 24; // length of first arm
	static const double l2 = 24; 
	static const double m1 = 00; // mass of first arm
	static const double m2 = 00; // mass of second arm
	static const double D1 = 00; // distance from center of mass to pivot
	static const double D2 = 00; // distance from center of mass of second arm to second pivot 
	static const double I1 = m1 * l1 * l1 / 3; // moment of interia of first arm (only) around first pivot
	static const double I2 = m2 * l2 * l2 / 3; // moment of interia of second arm around second pivot
	static const double Icom2 = m2 * l2 * l2 / 12; // moment of inertia of the second arm around the center of mass around the second pivot
};

