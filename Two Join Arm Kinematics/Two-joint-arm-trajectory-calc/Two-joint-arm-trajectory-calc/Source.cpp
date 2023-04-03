#include "armKinematics.h"
#include "TaskSpaceTrajectoryCalc.h"
#include "TwoJointArmProfiles.h"
#include <chrono>
#include <thread>
#include <iostream>

using namespace TwoJointArmConstants;
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
	TwoJointArmConstants::UPPER_ARM_LENGTH, TwoJointArmConstants::FOREARM_LENGTH+TwoJointArmConstants::EE_LENGTH/*, 2.0, 2.0*/};

	//double piecewiseLink = 0.572660801083; //1, 1
	double piecewiseLink = 0.584193463076; //0, 180

	for(int i = 0; i < 10; ++i)
	{
		for(int j = 0; j < 10; ++j)
		{
			if(i == j)
			{
				continue;
			}

			string fileName = to_string(i) + to_string(j) + ".csv";
			double theta = TwoJointArmConstants::ARM_POSITIONS[i][2];
			double phi = TwoJointArmConstants::ARM_POSITIONS[i][3];
			double x = TwoJointArmConstants::ARM_POSITIONS[j][0];
			double y = TwoJointArmConstants::ARM_POSITIONS[j][1];

			//double stowToCubeDist = 0.634465765372; //Original and working
			double stowToCubeDist = 0.704382227924; //New for bungee
			double autoStowToIntermediateAutoStow = 0.2792875869 + 0.0004;

			if(i == 0 && j == 1)
			{
				calc.generateCurvedTrajectory(fileName, theta, phi, x, y, stowToCubeDist, 01, 2.0, 2.0);
			}
			else if(i == 1 && j == 0)
			{
				calc.generateCurvedTrajectory(fileName, theta, phi, x, y, stowToCubeDist, -01, 2.0, 2.0);
			}
			else if(i == 0 && j == 2)
			{
				calc.generateCurvedTrajectory(fileName, theta, phi, x, y, 1.00222057943 + piecewiseLink, 02, 2.0, 2.0); //3j
			}
			else if (i == 2 && j == 0)
			{
				calc.generateCurvedTrajectory(fileName, theta, phi, x, y, 1.14635159681 + piecewiseLink, -02, 2.0, 2.0);
			}
			//else if (i == 0 && j == 3)
			//{
			//	calc.generateCurvedTrajectory(fileName, theta, phi, x, y, 1.191807202232 + piecewiseLink, 03, 2.5, 2.5); //4
			//}
			//else if (i == 3 && j == 0)
			//{
			//	calc.generateCurvedTrajectory(fileName, theta, phi, x, y, 1.191807202232 + piecewiseLink, -03, 2.5, 2.5);
			//}
			else if (i == 0 && j == 4)
			{
				calc.generateCurvedTrajectory(fileName, theta, phi, x, y, 1.52719714124 + piecewiseLink, 04, 2.0, 2.0);
			}
			else if (i == 4 && j == 0)
			{
				calc.generateCurvedTrajectory(fileName, theta, phi, x, y, 1.52719714124 + piecewiseLink, -04, 2.0, 2.0);
			}
			else if (i == 0 && j == 5)
			{
				calc.generateCurvedTrajectory(fileName, theta, phi, x, y, 0.738332377068 + piecewiseLink, 05, 2.0, 2.0);
			}
			else if (i == 5 && j == 0)
			{
				calc.generateCurvedTrajectory(fileName, theta, phi, x, y, 0.738332377068 + piecewiseLink, -05, 2.0, 2.0);
			}
			else if (i == 0 && j == 6)
			{
				calc.generateCurvedTrajectory(fileName, theta, phi, x, y, 1.2634213845 + piecewiseLink, 06, 2.0, 2.0);
			}
			else if (i == 6 && j == 0)
			{
				calc.generateCurvedTrajectory(fileName, theta, phi, x, y, 1.2634213845 + piecewiseLink, -06, 2.0, 2.0);
			}
			else if (i == 5 && j == 6)
			{
				calc.generateCurvedTrajectory(fileName, theta, phi, x, y, 0.548610709697, 56, 2.0, 2.0);
			}
			else if (i == 6 && j == 5)
			{
				calc.generateCurvedTrajectory(fileName, theta, phi, x, y, 0.548610709697, -56, 2.0, 2.0); 
			}
			else if (i == 0 && j == 7)
			{
				calc.generateCurvedTrajectory(fileName, theta, phi, x, y, 0.783824784682, 07, 2.0, 2.0);
			}//1.17801620928 for high ground, 0.783824784682 for cone intake
			else if (i == 7 && j == 0)
			{
				calc.generateCurvedTrajectory(fileName, theta, phi, x, y, 0.783824784682, -07, 2.0, 2.0);
			}
			else if (i == 0 && j == 8)
			{
				calc.generateCurvedTrajectory(fileName, theta, phi, x, y, 1.21225498066, 8, 2.0, 2.0);
			}
			else if (i == 8 && j == 0)
			{
				calc.generateCurvedTrajectory(fileName, theta, phi, x, y, 1.21225498066, -8, 2.0, 2.0);
			}
			else if (i == 0 && j == 9)
			{
				calc.generateCurvedTrajectory(fileName, theta, phi, x, y, 0.384657671138, 9, 2.0, 2.0);
			}
			else if(i == 9 && j == 0)
			{
				calc.generateCurvedTrajectory(fileName, theta, phi, x, y, 0.384657671138, -9, 2.0, 2.0);
			}
			else if (i == 2 && j == 9)
			{
				calc.generateAutoSpecialTrajectory(fileName, theta, phi, x, y, 0.6548126425 + 0.776400872421 + 0.0004,-29, 2.0, 2.0);
			}
			else if (i == 9 && j == 2)
			{
				calc.generateAutoSpecialTrajectory(fileName, theta, phi, x, y, 0.6548126425 + 0.776400872421 + 0.0004, 29, 2.0, 2.0);
			}//1.1695781018 + autoStowToIntermediateAutoStow
			else if (i == 4 && j == 9)
			{
				calc.generateAutoSpecialTrajectory(fileName, theta, phi, x, y, 1.18060206833 + 0.777220115957 + 0.0003, -49, 2.0, 2.0);
			}
			else if (i == 9 && j == 4)
			{
				calc.generateAutoSpecialTrajectory(fileName, theta, phi, x, y, 1.18060206833 + 0.777220115957 + 0.0003, 49, 2.0, 2.0);
			}//1.69536474602 + autoStowToIntermediateAutoStow
			else if (i == 5 && j == 9)
			{
				calc.generateAutoSpecialTrajectory(fileName, theta, phi, x, y, 0.506221602006 + 0.668800674773 + 0.0003, -59, 2.0, 2.0);
			}
			else if (i == 9 && j == 5)
			{
				calc.generateAutoSpecialTrajectory(fileName, theta, phi, x, y, 0.506221602006 + 0.668800674773 + 0.0003, 59, 2.0, 2.0);
			}//0.908515088341 + autoStowToIntermediateAutoStow
			else if (i == 6 && j == 9)
			{
			calc.generateAutoSpecialTrajectory(fileName, theta, phi, x, y, 0.955183569892 + 0.76299506825 - 0.0011, -69, 2.0, 2.0);
			}
			else if (i == 9 && j == 6)
			{
			calc.generateAutoSpecialTrajectory(fileName, theta, phi, x, y, 0.955183569892 + 0.76299506825 - 0.0011, 69, 2.0, 2.0);
			}//1.45385025101 + autoStowToIntermediateAutoStow
			else if (i == 7 && j == 9)
			{
			calc.generateAutoSpecialTrajectory(fileName, theta, phi, x, y, 0.826238522718 + 0.247504659514, -79, 2.0, 2.0);
			}
			else if (i == 9 && j == 7)
			{
			calc.generateAutoSpecialTrajectory(fileName, theta, phi, x, y, 0.826238522718 + 0.247504659514, 79, 2.0, 2.0);
			}//0.79440614813 + autoStowToIntermediateAutoStow
			else if(i == 4 && j == 3)
			{
				calc.generateSpecialTrajectory(fileName, theta, phi, x, y, 1.22022747337 + 0.880841786786 + stowToCubeDist + 0.0015 + 0.0018, -34, 3.0, 2.8);
			}
			else if (i == 3 && j == 4)
			{
				calc.generateSpecialTrajectory(fileName, theta, phi, x, y, 1.22022747337 + 0.880841786786 + stowToCubeDist + 0.0015 + 0.0018, 34, 3.0, 2.8);
			}
			else if(i == 2 && j == 3)
			{
				calc.generateSpecialTrajectory(fileName, theta, phi, x, y, 1.30119969351 + 0.278378573222 + stowToCubeDist - 0.007 + 0.0109, -23, 3.0, 2.8);
			}
			else if (i == 3 && j == 2)
			{
				calc.generateSpecialTrajectory(fileName, theta, phi, x, y, 1.30119969351 + 0.278378573222 + stowToCubeDist - 0.007 + 0.0109, 23, 3.0, 2.8);
			}
			else if (i == 5 && j == 3)
			{
				calc.generateSpecialTrajectory(fileName, theta, phi, x, y, 1.2221807912 + 0.0884840716942 + stowToCubeDist + 0.0004 + 0.00245, -53, 3.0, 2.8);
			}
			else if (i == 3 && j == 5)
			{
				calc.generateSpecialTrajectory(fileName, theta, phi, x, y, 1.2221807912 + 0.0884840716942 + stowToCubeDist + 0.0004 + 0.00245, 53, 3.0, 2.8);
			}
			else if (i == 6 && j == 3)
			{
				calc.generateSpecialTrajectory(fileName, theta, phi, x, y, 1.22538823408 + 0.63385732983 + stowToCubeDist + 0.0014 + 0.0019, -63, 3.0, 2.8);
			}
			else if (i == 3 && j == 6)
			{
				calc.generateSpecialTrajectory(fileName, theta, phi, x, y, 1.22538823408 + 0.63385732983 + stowToCubeDist + 0.0014 + 0.0019, 63, 3.0, 2.8);
			}
			else
			{
				calc.generateLinearTrajectory(fileName, theta, phi, x, y, 2.0, 2.0); //3
			}

			
		}
	}

	/*calc.generateLinearTrajectoryOptimized("02.csv", sTheta, sPhi, psX, psY);
	calc.generateLinearTrajectoryOptimized("01.csv", sTheta, sPhi, giX, giY);
	calc.generateLinearTrajectoryOptimized("02.csv", sTheta, sPhi, psX, psY);
	calc.generateLinearTrajectoryOptimized("03.csv", sTheta, sPhi, mX, mY);
	calc.generateLinearTrajectoryOptimized("04.csv", sTheta, sPhi, hX, hY);
	calc.generateLinearTrajectoryOptimized("05.csv", sTheta, sPhi, iX, iY);
	calc.generateLinearTrajectoryOptimized("10.csv", giTheta, giPhi, sX, sY);
	calc.generateLinearTrajectoryOptimized("12.csv", giTheta, giPhi, psX, psY);
	calc.generateLinearTrajectoryOptimized("13.csv", giTheta, giPhi, mX, mY);
	calc.generateLinearTrajectoryOptimized("14.csv", giTheta, giPhi, hX, hY);
	calc.generateLinearTrajectoryOptimized("15.csv", giTheta, giPhi, iX, iY);
	calc.generateLinearTrajectoryOptimized("20.csv", psTheta, psPhi, sX, sY);
	calc.generateLinearTrajectoryOptimized("21.csv", psTheta, psPhi, giX, giY);
	calc.generateLinearTrajectoryOptimized("23.csv", psTheta, psPhi, mX, mY);
	calc.generateLinearTrajectoryOptimized("24.csv", psTheta, psPhi, hX, hY);
	calc.generateLinearTrajectoryOptimized("25.csv", psTheta, psPhi, iX, iY);
	calc.generateLinearTrajectoryOptimized("30.csv", mTheta, mPhi, sX, sY);
	calc.generateLinearTrajectoryOptimized("31.csv", mTheta, mPhi, giX, giY);
	calc.generateLinearTrajectoryOptimized("32.csv", mTheta, mPhi, psX, psY);
	calc.generateLinearTrajectoryOptimized("34.csv", mTheta, mPhi, hX, hY);
	calc.generateLinearTrajectoryOptimized("35.csv", mTheta, mPhi, iX, iY);
	calc.generateLinearTrajectoryOptimized("40.csv", hTheta, hPhi, sX, sY);
	calc.generateLinearTrajectoryOptimized("41.csv", hTheta, hPhi, giX, giY);
	calc.generateLinearTrajectoryOptimized("42.csv", hTheta, hPhi, psX, psY);
	calc.generateLinearTrajectoryOptimized("43.csv", hTheta, hPhi, mX, mY);
	calc.generateLinearTrajectoryOptimized("45.csv", hTheta, hPhi, iX, iY);
	calc.generateLinearTrajectoryOptimized("50.csv", iTheta, iPhi, sX, sY);
	calc.generateLinearTrajectoryOptimized("51.csv", iTheta, iPhi, giX, giY);
	calc.generateLinearTrajectoryOptimized("52.csv", iTheta, iPhi, psX, psY);
	calc.generateLinearTrajectoryOptimized("53.csv", iTheta, iPhi, mX, mY);
	calc.generateLinearTrajectoryOptimized("54.csv", iTheta, iPhi, hX, hY);*/


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