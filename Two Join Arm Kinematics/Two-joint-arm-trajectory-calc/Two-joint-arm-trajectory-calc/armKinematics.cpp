#include "armKinematics.h"
#include "Helpers.h"
#include <iostream>

armKinematics::armKinematics()
{

}

pair<double, double> armKinematics::xyToAng(double x, double y, bool phiPositive)
{
	double radius = sqrt(x * x + y * y);
	if (x == 0 && y == 0)
	{
		return pair<double, double>{0, 0};
	}
	if (radius > TwoJointArmConstants::UPPER_ARM_LENGTH + TwoJointArmConstants::FOREARM_LENGTH)
	{
		return pair<double, double>{0, 0}; //TODO figure out
	}

	double phiCalc = (TwoJointArmConstants::UPPER_ARM_LENGTH * TwoJointArmConstants::UPPER_ARM_LENGTH + TwoJointArmConstants::FOREARM_LENGTH * TwoJointArmConstants::FOREARM_LENGTH
		- (radius * radius)) / (2 * TwoJointArmConstants::UPPER_ARM_LENGTH * TwoJointArmConstants::FOREARM_LENGTH);

	if (abs(phiCalc) > 1)
	{
		return pair<double, double>{0, 0};
	}

	double phi = acos(phiCalc);

	double returnPhi;
	if (phiPositive)
	{
		returnPhi = 180 - (phi * (180 / 3.1415));
	}
	else
	{
		returnPhi = (phi * (180 / 3.1415)) - 180;
	}


	double thetaCalc1 = (TwoJointArmConstants::FOREARM_LENGTH * sin(3.1415 - phi));
	if (phiPositive)
	{
		thetaCalc1 *= -1;
	}
	double thetaCalc2 = (TwoJointArmConstants::UPPER_ARM_LENGTH + TwoJointArmConstants::FOREARM_LENGTH * cos(3.1415 - phi));
	if (thetaCalc1 == 0 && thetaCalc2 == 0)
	{
		return pair<double, double>{0, 0};
	}
	double theta = atan2(y, x) - atan2(thetaCalc1, thetaCalc2);
	theta *= (180 / 3.1415);
	theta = 90 - theta;

	Helpers::normalizeAngle(theta);
	Helpers::normalizeAngle(returnPhi);
	return pair<double, double>{theta, returnPhi};
}

pair<double, double> armKinematics::angToXY(double theta, double phi)
{
	double upperArmX = -TwoJointArmConstants::UPPER_ARM_LENGTH * sin((-theta) * (3.1415 / 180));
	double upperArmY = TwoJointArmConstants::UPPER_ARM_LENGTH * cos((-theta) * (3.1415 / 180));

	double secondJointAng_baseCords = theta + phi;

	double forearmX = -TwoJointArmConstants::FOREARM_LENGTH * sin((-secondJointAng_baseCords) * (3.1415 / 180));
	double forearmY = TwoJointArmConstants::FOREARM_LENGTH * cos((-secondJointAng_baseCords) * (3.1415 / 180));

	return pair<double, double>{upperArmX + forearmX, upperArmY + forearmY};
}

pair<double, double> armKinematics::linVelToAngVel(double xVel, double yVel, double theta, double phi)
{
	double omega = theta + phi;
	pair<double, double> xy = angToXY(theta, phi);
	if (xy.first == 0 && xy.second == 0)
	{
		return pair<double, double>{0, 0};
	}
	double alpha = (3.1415 / 2) - atan2(xy.second, xy.first);


	double linVelPhi = 0;
	double linVelTheta = 0;
	if ((theta == 0 || theta == -180) && (phi == 0 || phi == -180))
	{
		//TODO what if purely vertical
	}
	else if (omega == 90 || omega == -90)
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
	double phiRadPerSec = linVelPhi / TwoJointArmConstants::FOREARM_LENGTH;

	return pair<double, double>{thetaRadPerSec * 180 / 3.1415, phiRadPerSec * 180 / 3.1415};

}
pair<double, double> armKinematics::angVelToLinVel(double thetaVel, double phiVel, double theta, double phi)
{
	double forearmVel = TwoJointArmConstants::FOREARM_LENGTH * phiVel * (3.1415 / 180);
	double forearmVelAng = theta + phi;
	double forearmXVel = forearmVel * cos(forearmVelAng * (3.1415 / 180));
	double forearmYVel = forearmVel * -sin(forearmVelAng * (3.1415 / 180));

	pair<double, double> xy = angToXY(theta, phi);
	double upperArmToEEDist = sqrt(xy.second * xy.second + xy.first * xy.first);
	double upperArmVel = upperArmToEEDist * thetaVel * (3.1415 / 180);
	if (xy.first == 0 && xy.second == 0)
	{
		return pair<double, double>{0, 0};
	}
	double upperArmToEEAng = (3.1415 / 2) - atan2(xy.second, xy.first);
	double upperArmXVel = upperArmVel * cos(upperArmToEEAng);
	double upperArmYVel = upperArmVel * -sin(upperArmToEEAng);

	double xVel = forearmXVel + upperArmXVel;
	double yVel = forearmYVel + upperArmYVel;

	return pair<double, double>{xVel, yVel};
}