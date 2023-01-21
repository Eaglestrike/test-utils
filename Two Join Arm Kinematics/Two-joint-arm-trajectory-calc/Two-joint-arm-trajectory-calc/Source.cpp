#include "armKinematics.h"
#include "TaskSpaceTrajectoryCalc.h"
#include "TwoJointArmProfiles.h"
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

	//Stowed,		 0.221, 0.26247, 18.84, 160.11
	//Ground intake, 0.40564, 0.53042, 59.59, 129.87
	//player station 0.54164, 0.43525, -29.64, 127.4
	//mid,			 1.02512, 0.29191, 20, 90
	//high,			 1.38078, 0.54399, 59.18, 16.16

	/*calc.generateLinearTrajectory("01.csv", 18.84, 160.11, 0.40564, 0.53042);
	calc.generateLinearTrajectory("02.csv", 18.84, 160.11, 0.54164, 0.43525);
	calc.generateLinearTrajectory("03.csv", 18.84, 160.11, 1.02512, 0.29191);
	calc.generateLinearTrajectory("04.csv", 18.84, 160.11, 1.38078, 0.54399);
	calc.generateLinearTrajectory("10.csv", 59.59, 129.87, 0.221, 0.26247);
	calc.generateLinearTrajectory("12.csv", 59.59, 129.87, 0.54164, 0.43525);
	calc.generateLinearTrajectory("13.csv", 59.59, 129.87, 1.02512, 0.29191);
	calc.generateLinearTrajectory("14.csv", 59.59, 129.87, 1.38078, 0.54399);
	calc.generateLinearTrajectory("20.csv", -29.64, 127.4, 0.221, 0.26247);
	calc.generateLinearTrajectory("21.csv", -29.64, 127.4, 0.40564, 0.53042);
	calc.generateLinearTrajectory("23.csv", -29.64, 127.4, 1.02512, 0.29191);
	calc.generateLinearTrajectory("24.csv", -29.64, 127.4, 1.38078, 0.54399);
	calc.generateLinearTrajectory("30.csv", 20, 90, 0.221, 0.26247);
	calc.generateLinearTrajectory("31.csv", 20, 90, 0.40564, 0.53042);
	calc.generateLinearTrajectory("32.csv", 20, 90, 0.54164, 0.43525);
	calc.generateLinearTrajectory("34.csv", 20, 90, 1.38078, 0.54399);
	calc.generateLinearTrajectory("40.csv", 59.18, 16.16, 0.221, 0.26247);
	calc.generateLinearTrajectory("41.csv", 59.18, 16.16, 0.40564, 0.53042);
	calc.generateLinearTrajectory("42.csv", 59.18, 16.16, 0.54164, 0.43525);
	calc.generateLinearTrajectory("43.csv", 59.18, 16.16, 1.02512, 0.29191);*/



	TwoJointArmProfiles profiles;
	profiles.readProfiles();

	int b = 0;

	std::pair<double, double> maxTorque = profiles.returnMaxTorque();

	std::cout << "max torque 1: " << maxTorque.first << ", max torque 2: " << maxTorque.second << "\n";

}