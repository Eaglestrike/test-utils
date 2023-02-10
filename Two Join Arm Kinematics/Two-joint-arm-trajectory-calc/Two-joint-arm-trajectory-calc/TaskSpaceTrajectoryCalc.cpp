#include "TaskSpaceTrajectoryCalc.h"
#include <fstream>

TaskSpaceTrajectoryCalc::TaskSpaceTrajectoryCalc(double SHOULDER_ARM_MAX_V, double SHOULDER_ARM_MAX_A, double ELBOW_ARM_MAX_V, double ELBOW_ARM_MAX_A, double SHOULDER_ARM_LENGTH, double ELBOW_ARM_LENGTH, double TASK_MAX_V, double TASK_MAX_A) :
	shoulderArmMaxV_(SHOULDER_ARM_MAX_V), shoulderArmMaxA_(SHOULDER_ARM_MAX_A), elbowArmMaxV_(ELBOW_ARM_MAX_V), elbowArmMaxA_(ELBOW_ARM_MAX_A), shoulderArmLength_(SHOULDER_ARM_LENGTH), elbowArmLength_(ELBOW_ARM_LENGTH), taskMaxV_(TASK_MAX_V), taskMaxA_(TASK_MAX_A)
{

}


void TaskSpaceTrajectoryCalc::generateLinearTrajectory(string file_name, double startTheta, double startPhi, double x, double y)
{
	pair<double, double> startXY = armKinematics::angToXY(startTheta, startPhi);
	bool phiPositive = (startPhi >= 0);

	if(sqrt(x * x + y * y) > shoulderArmLength_ + elbowArmLength_)
	{
		return;
	}

	double deltaX = x - startXY.first;
	double deltaY = y - startXY.second;

	double distance = sqrt(deltaX * deltaX + deltaY * deltaY);
	if(deltaY == 0 && deltaX == 0)
	{
		return;
	}
	double angle = atan2(deltaY, deltaX);

	TrajectoryCalc trajectoryCalc{ taskMaxV_, taskMaxA_, 0, 0, 0, 0};
	trajectoryCalc.generateTrajectory(0, distance, 0);
	
	double totalTime = trajectoryCalc.getTotalTime();
	double xVel, yVel, wantedX, wantedY, xAcc, yAcc;
	pair<double, double> angVel;
	pair<double, double> angAcc;

	pair<double, double> angAccTest;
	pair<double, double> prevAngVel{ 0, 0 };

	pair<double, double> angles;
	pair<double, double> testPos{startTheta, startPhi};
	ofstream outfile(file_name);

	double dt = 0.001;
	for(double i = 0; i < totalTime; i += dt)
	{
		tuple<double, double, double> profile = trajectoryCalc.getProfile(i); //a, v, p
		xVel = get<1>(profile) * cos(angle);
		yVel = get<1>(profile) * sin(angle);
		xAcc = get<0>(profile) * cos(angle);
		yAcc = get<0>(profile) * sin(angle);
		wantedX = get<2>(profile) * cos(angle) + startXY.first;
		wantedY = get<2>(profile) * sin(angle) + startXY.second;
		angles = armKinematics::xyToAng(wantedX, wantedY, phiPositive);

		angVel = armKinematics::linVelToAngVel(xVel, yVel, angles.first, angles.second);
		//angAcc = armKinematics::linVelToAngVel(xAcc, yAcc, angles.first, angles.second); //TODO not correct

		angAccTest = { (angVel.first - prevAngVel.first) / dt, (angVel.second - prevAngVel.second) / dt };
		prevAngVel = angVel;

		//testPos = { testPos.first + angVel.first * dt, testPos.second + angVel.second * dt };
		//cout << angVel.first << ", " << testPos.first << endl;

		/*outfile << i << ", " << angles.first << ", " << angles.second << ", " << angVel.first << ", " << angVel.second << ", " << angAcc.first << ", " << angAcc.second <<
		", " << wantedX << ", " << wantedY << ", " << xVel << ", " << yVel << ", " << xAcc << ", " << yAcc << ", " << angAccTest.first << ", " << angAccTest.second << 
		", " << testPos.first << ", " << testPos.second << endl;*/

		outfile << i << ", " << angles.first << ", " << angles.second << ", " << angVel.first << ", " << angVel.second << ", " << angAccTest.first << ", " << angAccTest.second << endl;
	}

	outfile.close();
	cout << file_name + " completed" << endl;
}

void TaskSpaceTrajectoryCalc::generateCurvedTrajectory(string file_name, double startTheta, double startPhi, double x, double y, double distance, int type)
{
	pair<double, double> startXY = armKinematics::angToXY(startTheta, startPhi);
	bool phiPositive = (startPhi >= 0);
	//startXY.first += 0.05;

	if (sqrt(x * x + y * y) > shoulderArmLength_ + elbowArmLength_)
	{
		return;
	}

	//double distance = 1.138; //Number
	if (x == startXY.first && y == startXY.second)
	{
		return;
	}

	TrajectoryCalc trajectoryCalc{ taskMaxV_, taskMaxA_, 0, 0, 0, 0 };
	trajectoryCalc.generateTrajectory(0, distance, 0);

	double totalTime = trajectoryCalc.getTotalTime();
	double xVel, yVel, xAcc, yAcc;
	double wantedX = startXY.first;
	double wantedY = startXY.second;
	pair<double, double> angVel;
	pair<double, double> angAcc;

	pair<double, double> angAccTest;
	pair<double, double> prevAngVel{ 0, 0 };

	pair<double, double> angles;
	pair<double, double> testPos{ startTheta, startPhi };
	ofstream outfile(file_name);

	double dt = 0.0001;

	for (double i = 0; i < totalTime; i += dt)
	{
		tuple<double, double, double> profile = trajectoryCalc.getProfile(i); //a, v, p
		
		double slope = getSlope(type, wantedX);
			
		double angle = atan2(slope, 1);
		//cout << i << ", " << sqrt(wantedX * wantedX + wantedY * wantedY) << ", " << wantedX << ", " << wantedY << endl;

		//wantedX = get<2>(profile) * cos(angle) + startXY.first;
		//wantedY = get<2>(profile) * sin(angle) + startXY.second;

		xVel = get<1>(profile) * cos(angle);
		yVel = get<1>(profile) * sin(angle);
		//xAcc = get<0>(profile) * cos(angle);
		//yAcc = get<0>(profile) * sin(angle);

		if(type < 0)
		{
			xVel *= -1;
			yVel *= -1;
		}

		angles = armKinematics::xyToAng(wantedX, wantedY, phiPositive);

		angVel = armKinematics::linVelToAngVel(xVel, yVel, angles.first, angles.second);
		//angAcc = armKinematics::linVelToAngVel(xAcc, yAcc, angles.first, angles.second); //TODO not correct

		angAccTest = { (angVel.first - prevAngVel.first) / dt, (angVel.second - prevAngVel.second) / dt };
		prevAngVel = angVel;

		testPos = { testPos.first + angVel.first * dt, testPos.second + angVel.second * dt };
		//cout << angVel.first << ", " << testPos.first << endl;

		/*outfile << i << ", " << angles.first << ", " << angles.second << ", " << angVel.first << ", " << angVel.second << ", " << angAcc.first << ", " << angAcc.second <<
		", " << wantedX << ", " << wantedY << ", " << xVel << ", " << yVel << ", " << xAcc << ", " << yAcc << ", " << angAccTest.first << ", " << angAccTest.second <<
		", " << testPos.first << ", " << testPos.second << endl;*/

		outfile << i << ", " << angles.first << ", " << angles.second << ", " << angVel.first << ", " << angVel.second << ", " << angAccTest.first << ", " << angAccTest.second << ", " << testPos.first << ", " << testPos.second << ", " << wantedX << ", " << wantedY << endl;

		wantedX += xVel * dt;
		wantedY += yVel * dt;
	}

	outfile.close();
	cout << file_name + " completed" << endl;
}

double TaskSpaceTrajectoryCalc::getSlope(int type, double x)
{
	switch(type)
	{
	case 1:
	{
		return 3.0 / (4 * sqrt(5) * sqrt(x - (61.0 / 200.0)));
	}
	case -1:
	{
		return 3.0 / (4 * sqrt(5) * sqrt(x - (61.0 / 200.0)));
	}
	}
}

void TaskSpaceTrajectoryCalc::generateLinearTrajectoryOptimized(string file_name, double startTheta, double startPhi, double endX, double endY)
{
	pair<double, double> startXY = armKinematics::angToXY(startTheta, startPhi);
	bool phiPositive = (startPhi >= 0);

	if (sqrt(endX * endX + endY * endY) > shoulderArmLength_ + elbowArmLength_)
	{
		return;
	}

	double deltaX = endX - startXY.first;
	double deltaY = endY - startXY.second;

	double distance = sqrt(deltaX * deltaX + deltaY * deltaY);
	if (deltaY == 0 && deltaX == 0)
	{
		return;
	}
	double angle = atan2(deltaY, deltaX);
	double xVelRatio = cos(angle);
	double yVelRatio = sin(angle);

	double testX = startXY.first;
	double testY = startXY.second;
	pair<double, double> testAngs = armKinematics::xyToAng(testX, testY, phiPositive);

	bool changesThetaDir = false;
	pair<double, double> startAngVel = armKinematics::linVelToAngVel(xVelRatio, yVelRatio, startTheta, startPhi);
	bool thetaStartPositive = (startAngVel.first >= 0);
	bool phiStartPositive = (startAngVel.second >= 0);
	double thetaChangeX = 0;

	while(abs(testX - endX) > 0.001)
	{
		testAngs = armKinematics::xyToAng(testX, testY, phiPositive);
		pair<double, double> angVels = armKinematics::linVelToAngVel(xVelRatio, yVelRatio, testAngs.first, testAngs.second);
		
		if(((angVels.first > 0) && !thetaStartPositive) || ((angVels.first <= 0) && thetaStartPositive))
		{
			changesThetaDir = true;
			thetaChangeX = testX;
			break;
		}

		testX += xVelRatio * 0.0001;
		testY += yVelRatio * 0.0001;
	}

	//cout << changesThetaDir << ", " << thetaStartPositive << ", ";
	//WORKING TILL HERE

	
	

	
	//outfile << i << ", " << angles.first << ", " << angles.second << ", " << angVel.first << ", " << angVel.second << ", " << angAcc.first << ", " << angAcc.second << endl;

	//outfile.close();
	cout << file_name + " completed" << endl;
	//cout << file_name << " complete" << endl;
}

//map<double, tuple<pair<double, double>, pair<double, double>, pair<double, double>>> TaskSpaceTrajectoryCalc::generateLinMaxAccelTrajectory(pair<double, double> startXY, pair<double, double> startAng, pair<double, double> startAngVel, pair<double, double> endXY)
//{
//	const double dt = 0.0001;
//
//	double currX = startXY.first;
//	double currY = startXY.second;
//	double currTheta = startAng.first;
//	double currPhi = startAng.second;
//	//double currXVel = 0;
//	//double currYVel = 0;
//	double currThetaVel = startAngVel.first;
//	double currPhiVel = startAngVel.second;
//	double currThetaAcc = 0;
//	double currPhiAcc = 0;
//
//	double wantedEndX = endXY.first;
//
//	double deltaX = endXY.first - startXY.first;
//	double deltaY = endXY.second - startXY.second;
//
//	double angle = atan2(deltaY, deltaX);
//	double xVelRatio = cos(angle);
//	double yVelRatio = sin(angle);
//
//	pair<double, double> angVel = armKinematics::linVelToAngVel(xVelRatio, yVelRatio, currTheta, currPhi);
//	bool thetaStartVelPositive = (angVel.first >= 0);
//	bool phiStartVelPositive = (angVel.second >= 0);
//
//	for (double i = 0; abs(currX - wantedEndX) > 0.001; i += dt)
//	{
//		angVel = armKinematics::linVelToAngVel(xVelRatio, yVelRatio, currTheta, currPhi);
//		double thetaToPhiVelRatio = angVel.first / angVel.second;
//
//		double maxThetaVel, maxPhiVel, minThetaVel, minPhiVel;
//
//		maxThetaVel = currThetaVel + dt * TwoJointArmConstants::SHOULDER_ARM_MAX_ACC;
//		if (maxThetaVel > TwoJointArmConstants::SHOULDER_ARM_MAX_VEL)
//		{
//			maxThetaVel = TwoJointArmConstants::SHOULDER_ARM_MAX_VEL;
//		}
//		else if (maxThetaVel < -TwoJointArmConstants::SHOULDER_ARM_MAX_VEL)
//		{
//			maxThetaVel = -TwoJointArmConstants::SHOULDER_ARM_MAX_VEL;
//		}
//		minThetaVel = currThetaVel - dt * TwoJointArmConstants::SHOULDER_ARM_MAX_ACC;
//		if (minThetaVel > TwoJointArmConstants::SHOULDER_ARM_MAX_VEL)
//		{
//			minThetaVel = TwoJointArmConstants::SHOULDER_ARM_MAX_VEL;
//		}
//		else if (minThetaVel < -TwoJointArmConstants::SHOULDER_ARM_MAX_VEL)
//		{
//			minThetaVel = -TwoJointArmConstants::SHOULDER_ARM_MAX_VEL;
//		}
//
//		maxPhiVel = currPhiVel + dt * TwoJointArmConstants::ELBOW_ARM_MAX_ACC;
//		if (maxPhiVel > TwoJointArmConstants::ELBOW_ARM_MAX_VEL)
//		{
//			maxPhiVel = TwoJointArmConstants::ELBOW_ARM_MAX_VEL;
//		}
//		else if (maxPhiVel < -TwoJointArmConstants::ELBOW_ARM_MAX_VEL)
//		{
//			maxPhiVel = -TwoJointArmConstants::ELBOW_ARM_MAX_VEL;
//		}
//		minPhiVel = currPhiVel - dt * TwoJointArmConstants::ELBOW_ARM_MAX_ACC;
//		if (minPhiVel > TwoJointArmConstants::ELBOW_ARM_MAX_VEL)
//		{
//			minPhiVel = TwoJointArmConstants::ELBOW_ARM_MAX_VEL;
//		}
//		else if (minPhiVel < -TwoJointArmConstants::ELBOW_ARM_MAX_VEL)
//		{
//			minPhiVel = -TwoJointArmConstants::ELBOW_ARM_MAX_VEL;
//		}
//
//		double possibleThetaVel;
//		if (phiStartVelPositive)
//		{
//			possibleThetaVel = thetaToPhiVelRatio * maxPhiVel;
//		}
//		else
//		{
//			possibleThetaVel = thetaToPhiVelRatio * minPhiVel;
//		}
//
//		if (possibleThetaVel > maxThetaVel || possibleThetaVel < minThetaVel)
//		{
//			if (thetaStartVelPositive)
//			{
//				currThetaVel = maxThetaVel;
//				currThetaAcc = TwoJointArmConstants::SHOULDER_ARM_MAX_ACC;
//			}
//			else
//			{
//				currThetaVel = minThetaVel;
//				currThetaAcc = -TwoJointArmConstants::SHOULDER_ARM_MAX_ACC;
//			}
//
//			double tempPhiVel = currPhiVel;//Here, just keeps accelerating phi
//			currPhiVel = currThetaVel / thetaToPhiVelRatio;
//			currPhiAcc = (currPhiVel - tempPhiVel) / dt;
//		}
//		else
//		{
//			double tempThetaVel = currThetaVel;
//			currThetaVel = possibleThetaVel;
//			currThetaAcc = (possibleThetaVel - tempThetaVel) / dt;
//			if (phiStartVelPositive)
//			{
//				currPhiVel = maxPhiVel;
//				currPhiVel = TwoJointArmConstants::ELBOW_ARM_MAX_ACC;
//			}
//			else
//			{
//				currPhiVel = minPhiVel;
//				currPhiAcc = -TwoJointArmConstants::ELBOW_ARM_MAX_ACC;
//			}
//		}
//
//		pair<double, double> xyVel = armKinematics::angVelToLinVel(currThetaVel, currPhiVel, currTheta, currPhi);
//		currX += xyVel.first * dt;
//		currY += xyVel.second * dt;
//
//		currTheta += currThetaVel * dt;
//		currPhi += currPhiVel * dt;
//
//		cout << currTheta << ", " << currPhi << ", " << currThetaVel << ", " << currPhiVel << endl;
//
//
//	}
//}