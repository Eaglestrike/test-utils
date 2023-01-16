#include "armKinematics.h"
#include "Helpers.h"
#include <iostream>

armKinematics::armKinematics()
{

}

std::pair<double, double> armKinematics::xyToAng(double x, double y, bool phiPositive)
{
	double radius = sqrt(x * x + y * y);
	if (x == 0 && y == 0)
	{
		return std::pair<double, double>{0, 0};
	}
	if(radius > TwoJointArmConstants::LOWER_ARM_LENGTH + TwoJointArmConstants::UPPER_ARM_LENGTH)
	{
		return std::pair<double, double>{0, 0}; //TODO figure out
	}

	double phiCalc = (TwoJointArmConstants::LOWER_ARM_LENGTH * TwoJointArmConstants::LOWER_ARM_LENGTH + TwoJointArmConstants::UPPER_ARM_LENGTH * TwoJointArmConstants::UPPER_ARM_LENGTH
	- (radius * radius)) / (2 * TwoJointArmConstants::LOWER_ARM_LENGTH * TwoJointArmConstants::UPPER_ARM_LENGTH);

	if(abs(phiCalc) > 1)
	{
		return std::pair<double, double>{0, 0};
	}

	double phi = acos(phiCalc);

	double returnPhi;
	if(phiPositive)
	{
		returnPhi = 180 - (phi * (180 / 3.1415));
	}
	else
	{
		returnPhi = (phi * (180 / 3.1415)) - 180;
	}


	double thetaCalc1 = (TwoJointArmConstants::UPPER_ARM_LENGTH * sin(3.1415 - phi));
	if (phiPositive)
	{
		thetaCalc1 *= -1;
	}
	double thetaCalc2 = (TwoJointArmConstants::LOWER_ARM_LENGTH + TwoJointArmConstants::UPPER_ARM_LENGTH * cos(3.1415 - phi));
	if(thetaCalc1 == 0 && thetaCalc2 == 0)
	{
		return std::pair<double, double>{0, 0};
	}
	double theta = atan2(y, x) - atan2(thetaCalc1, thetaCalc2);
	theta *= (180 / 3.1415);
	theta = 90 - theta;

	Helpers::normalizeAngle(theta);
	Helpers::normalizeAngle(returnPhi);
	return std::pair<double, double>{theta, returnPhi};
}

std::pair<double, double> armKinematics::angToXY(double theta, double phi)
{
	double lowerArmX = -TwoJointArmConstants::LOWER_ARM_LENGTH * sin((-theta) * (3.1415 / 180));
	double lowerArmY = TwoJointArmConstants::LOWER_ARM_LENGTH * cos((-theta) * (3.1415 / 180));

	double secondJointAng_baseCords = theta + phi;

	double upperArmX = -TwoJointArmConstants::UPPER_ARM_LENGTH * sin((-secondJointAng_baseCords) * (3.1415 / 180));
	double upperArmY = TwoJointArmConstants::UPPER_ARM_LENGTH * cos((-secondJointAng_baseCords) * (3.1415 / 180));

	return std::pair<double, double>{lowerArmX + upperArmX, lowerArmY + upperArmY};
}

std::pair<double, double> armKinematics::linVelToAngVel(double xVel, double yVel, double theta, double phi)
{
	double omega = theta + phi;
	std::pair<double, double> xy = angToXY(theta, phi);
	if(xy.first == 0 && xy.second == 0)
	{
		return std::pair<double, double>{0, 0};
	}
	double alpha = (3.1415 / 2) - atan2(xy.second, xy.first);


	double linVelPhi = 0;
	double linVelTheta = 0;
	if ((theta == 0 || theta == -180) && (phi == 0 || phi == -180))
	{
		//TODO what if purely vertical
	}
	else if(omega == 90 || omega == -90)
	{
		omega *= (3.1415 / 180);
		linVelPhi = (-yVel - (xVel * tan(alpha))) / (-cos(omega) * tan(alpha) + sin(omega));
		linVelTheta = (xVel - linVelPhi * cos(omega)) / cos(alpha);
	}
	else
	{
		omega *= (3.1415 / 180);
		linVelTheta = (-yVel - (xVel * tan(omega))) / (-cos(alpha) * tan(omega) + sin(alpha));
		linVelPhi = (xVel - linVelTheta * cos(alpha)) / cos(omega);
	}

	double thetaRadPerSec = linVelTheta / (sqrt(xy.first * xy.first + xy.second * xy.second));
	double phiRadPerSec = linVelPhi / TwoJointArmConstants::UPPER_ARM_LENGTH;

	return std::pair<double, double>{thetaRadPerSec * 180 / 3.1415, phiRadPerSec * 180 / 3.1415};

}
std::pair<double, double> armKinematics::angVelToLinVel(double thetaVel, double phiVel, double theta, double phi)
{
	double upperArmVel = TwoJointArmConstants::UPPER_ARM_LENGTH * phiVel * (3.1415 / 180);
	double upperArmVelAng = theta + phi;
	double upperArmXVel = upperArmVel * cos(upperArmVelAng * (3.1415 / 180));
	double upperArmYVel = upperArmVel * -sin(upperArmVelAng * (3.1415 / 180));

	std::pair<double, double> xy = angToXY(theta, phi);
	double lowerArmToEEDist = sqrt(xy.second * xy.second + xy.first * xy.first);
	double lowerArmVel = lowerArmToEEDist * thetaVel * (3.1415 / 180);
	if(xy.first == 0 && xy.second == 0)
	{
		return std::pair<double, double>{0, 0};
	}
	double lowerArmToEEAng = (3.1415 / 2) - atan2(xy.second, xy.first);
	double lowerArmXVel = lowerArmVel * cos(lowerArmToEEAng);
	double lowerArmYVel = lowerArmVel * -sin(lowerArmToEEAng);

	double xVel = upperArmXVel + lowerArmXVel;
	double yVel = upperArmYVel + lowerArmYVel;
	
	return std::pair<double, double>{xVel, yVel};
}