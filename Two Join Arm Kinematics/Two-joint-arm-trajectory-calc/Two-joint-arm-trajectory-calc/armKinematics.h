#pragma once
#include "constants.h"
#include <tuple>

#define M_PI 3.141592653

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
	static constexpr double l1 = 0.635; // length of first arm
	static constexpr double l2 = 1.016; 
	static constexpr double m1 = 2.62; // mass of first arm
	static constexpr double m2 = 4.18; // mass of second arm
	static constexpr double D1 = 0.317; // distance from center of mass of first arm to proximal pivot (TODO GET ACTUAL VALUE)
	static constexpr double D2 = 0.508; // distance from center of mass of second arm to distal pivot (TODO GET ACTUAL VALUE)
	static constexpr double I1 = m1 * l1 * l1 / 3; // moment of interia of first arm (only) around proximal pivot
	static constexpr double I2 = m2 * l2 * l2 / 3; // moment of interia of second arm around distal pivot
	static constexpr double Icom2 = m2 * l2 * l2 / 12; // moment of inertia of the second arm around its center of mass
};

