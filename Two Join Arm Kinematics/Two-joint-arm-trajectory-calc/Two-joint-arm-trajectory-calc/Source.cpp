#include "armKinematics.h"
#include "TaskSpaceTrajectoryCalc.h"
#include "TwoJointArmProfiles.h"
#include "TwoJointArm.h"
#include <chrono>
#include <thread>
#include <iostream>

using namespace std;
using namespace std::this_thread; // sleep_for, sleep_until
using namespace std::chrono; // nanoseconds, system_clock, seconds

//std::pair<double, double> xyToAng(double x, double y, bool jointUp);
//std::pair<double, double> angToXY(double theta, double phi);
//
//std::pair<double, double> linVelToAngVel(double xVel, double yVel, double theta, double phi);
//std::pair<double, double> angVelToLinVel(double thetaVel, double phiVel, double theta, double phi);

int main()
{
	// ---------------------- ARM KINEMATICS TEST ---------------------------
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


	// ---------------------- TRAJECTORY GENERATOR ---------------------------
	TaskSpaceTrajectoryCalc calc{TwoJointArmConstants::SHOULDER_ARM_MAX_VEL, TwoJointArmConstants::SHOULDER_ARM_MAX_ACC, TwoJointArmConstants::ELBOW_ARM_MAX_VEL, TwoJointArmConstants::ELBOW_ARM_MAX_ACC,
	TwoJointArmConstants::UPPER_ARM_LENGTH, TwoJointArmConstants::FOREARM_LENGTH, 1.0, 1.0 };

	//Stowed
	double sX = 0.355439218;
	double sY = -0.184396634;
	double sTheta = 6.42;
	double sPhi = 154.35;

	//Ground intake
	double giX = 0.40564;
	double giY = -0.53042;
	double giTheta = 59.59;
	double giPhi = 129.87;

	//player station
	double psX = 0.54164;
	double psY = 0.43525;
	double psTheta = -29.64;
	double psPhi = 127.4;

	//mid
	double mX = 1.02512;
	double mY = 0.29191;
	double mTheta = 20;
	double mPhi = 90;

	//high
	double hX = 1.38078;
	double hY = 0.54399;
	double hTheta = 59.18;
	double hPhi = 16.16;

	//intake
	double iX = 0.33037399;
	double iY = -0.537556202;
	double iTheta = 62.4;
	double iPhi = 133.21;


	calc.generateLinearTrajectory("01.csv", sTheta, sPhi, giX, giY);
	calc.generateLinearTrajectory("02.csv", sTheta, sPhi, psX, psY);
	calc.generateLinearTrajectory("03.csv", sTheta, sPhi, mX, mY);
	calc.generateLinearTrajectory("04.csv", sTheta, sPhi, hX, hY);
	calc.generateLinearTrajectory("05.csv", sTheta, sPhi, iX, iY);
	calc.generateLinearTrajectory("10.csv", giTheta, giPhi, sX, sY);
	calc.generateLinearTrajectory("12.csv", giTheta, giPhi, psX, psY);
	calc.generateLinearTrajectory("13.csv", giTheta, giPhi, mX, mY);
	calc.generateLinearTrajectory("14.csv", giTheta, giPhi, hX, hY);
	calc.generateLinearTrajectory("15.csv", giTheta, giPhi, iX, iY);
	calc.generateLinearTrajectory("20.csv", psTheta, psPhi, sX, sY);
	calc.generateLinearTrajectory("21.csv", psTheta, psPhi, giX, giY);
	calc.generateLinearTrajectory("23.csv", psTheta, psPhi, mX, mY);
	calc.generateLinearTrajectory("24.csv", psTheta, psPhi, hX, hY);
	calc.generateLinearTrajectory("25.csv", psTheta, psPhi, iX, iY);
	calc.generateLinearTrajectory("30.csv", mTheta, mPhi, sX, sY);
	calc.generateLinearTrajectory("31.csv", mTheta, mPhi, giX, giY);
	calc.generateLinearTrajectory("32.csv", mTheta, mPhi, psX, psY);
	calc.generateLinearTrajectory("34.csv", mTheta, mPhi, hX, hY);
	calc.generateLinearTrajectory("35.csv", mTheta, mPhi, iX, iY);
	calc.generateLinearTrajectory("40.csv", hTheta, hPhi, sX, sY);
	calc.generateLinearTrajectory("41.csv", hTheta, hPhi, giX, giY);
	calc.generateLinearTrajectory("42.csv", hTheta, hPhi, psX, psY);
	calc.generateLinearTrajectory("43.csv", hTheta, hPhi, mX, mY);
	calc.generateLinearTrajectory("45.csv", hTheta, hPhi, iX, iY);
	calc.generateLinearTrajectory("50.csv", iTheta, iPhi, sX, sY);
	calc.generateLinearTrajectory("51.csv", iTheta, iPhi, giX, giY);
	calc.generateLinearTrajectory("52.csv", iTheta, iPhi, psX, psY);
	calc.generateLinearTrajectory("53.csv", iTheta, iPhi, mX, mY);
	calc.generateLinearTrajectory("54.csv", iTheta, iPhi, hX, hY);



	// ---------------------- READING PROFILES ---------------------------
	//TwoJointArmProfiles profiles;
	//profiles.readProfiles();

	//pair<TwoJointArmProfiles::Positions, TwoJointArmProfiles::Positions> testKey{ TwoJointArmProfiles::STOWED, TwoJointArmProfiles::HIGH };
	//cout << get<1>(profiles.getOmegaProfile(testKey, 0.1)) << endl;


	// ---------------------- ARM SIM ---------------------------
	/*TwoJointArm arm;

	double time = 0.0;
	cout << "Periodic started" << endl;
	while (true)
	{
		cout << time << ", " << arm.getState() << endl;

		arm.perioidc(time);

		if (time > 0)
		{
			arm.setPosTo(TwoJointArmProfiles::PLAYER_STATION);
		}
		//if (time > 3.6 && time < 3.61)
		//{
		//	arm.setPosTo(TwoJointArmProfiles::HIGH);
		//}

		time += 0.005;
		sleep_until(system_clock::now() + 0.001s);

	}*/


	// ---------------------- ARM MATH TESTER ---------------------------
	/*TwoJointArm arm;
	double theta = 0;
	double phi = 90;
	bool hasCone = false;

	cout << "Shoulder G Torque: " << arm.calcShoulderGravityTorque(theta, phi, hasCone) << endl;
	cout << "Elbow G Torque: " << arm.calcElbowGravityTorque(theta, phi, hasCone) << endl;
	cout << "Shoulder I: " << arm.calcShoulderRotInertia(phi, hasCone) << endl;
	cout << "Elbow I: " << arm.calcElbowRotInertia(hasCone) << endl;*/

}