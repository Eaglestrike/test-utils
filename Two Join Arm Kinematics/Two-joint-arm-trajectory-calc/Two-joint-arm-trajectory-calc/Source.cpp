#include "armKinematics.h"
#include "TaskSpaceTrajectoryCalc.h"
#include <iostream>

using namespace std;

//std::pair<double, double> xyToAng(double x, double y, bool jointUp);
//std::pair<double, double> angToXY(double theta, double phi);
//
//std::pair<double, double> linVelToAngVel(double xVel, double yVel, double theta, double phi);
//std::pair<double, double> angVelToLinVel(double thetaVel, double phiVel, double theta, double phi);

int main()
{
	/*pair<double, double> ang = armKinematics::xyToAng(1, 1, false);
	cout << ang.first << ", " << ang.second << endl;

	pair<double, double> xy = armKinematics::angToXY(ang.first, ang.second);
	cout << xy.first << ", " << xy.second << endl;

	double theta = 0;
	double phi = 90;
	pair<double, double> angVel = armKinematics::linVelToAngVel(0.5, -0.421, theta, phi);
	cout << angVel.first << ", " << angVel.second << endl;

	pair<double, double> linVel = armKinematics::angVelToLinVel(angVel.first, angVel.second, theta, phi);
	cout << linVel.first << ", " << linVel.second << endl;*/

	TaskSpaceTrajectoryCalc calc{ TwoJointArmConstants::LOWER_ARM_MAX_VEL, TwoJointArmConstants::LOWER_ARM_MAX_ACC, TwoJointArmConstants::UPPER_ARM_MAX_VEL, TwoJointArmConstants::UPPER_ARM_MAX_ACC,
	TwoJointArmConstants::LOWER_ARM_LENGTH, TwoJointArmConstants::UPPER_ARM_LENGTH, 0.5, 0.25 };

	calc.generateLinearTrajectory("yeah.csv", -8, 166, 1.3208, 0.5842);

}