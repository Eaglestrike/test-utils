#include "TaskSpaceTrajectoryCalc.h"
#include <fstream>

TaskSpaceTrajectoryCalc::TaskSpaceTrajectoryCalc(double SHOULDER_ARM_MAX_V, double SHOULDER_ARM_MAX_A, double ELBOW_ARM_MAX_V, double ELBOW_ARM_MAX_A, double SHOULDER_ARM_LENGTH, double ELBOW_ARM_LENGTH/*, double TASK_MAX_V, double TASK_MAX_A*/) :
	shoulderArmMaxV_(SHOULDER_ARM_MAX_V), shoulderArmMaxA_(SHOULDER_ARM_MAX_A), elbowArmMaxV_(ELBOW_ARM_MAX_V), elbowArmMaxA_(ELBOW_ARM_MAX_A), shoulderArmLength_(SHOULDER_ARM_LENGTH), elbowArmLength_(ELBOW_ARM_LENGTH)/*, taskMaxV_(TASK_MAX_V), taskMaxA_(TASK_MAX_A)*/
{

}


void TaskSpaceTrajectoryCalc::generateLinearTrajectory(string file_name, double startTheta, double startPhi, double x, double y, double maxV, double maxA)
{
	pair<double, double> startXY = armKinematics::angToXY(startTheta, startPhi);
	bool phiPositive = (startPhi >= 0);

	double maxThetaAcc = 0;
	double maxThetaVel = 0;
	double maxPhiAcc = 0;
	double maxPhiVel = 0;

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

	TrajectoryCalc trajectoryCalc{ maxV, maxA, 0, 0, 0, 0};
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

		if (abs(angVel.first) > maxThetaVel)
		{
			maxThetaVel = abs(angVel.first);
		}
		if (abs(angVel.second) > maxPhiVel)
		{
			maxPhiVel = abs(angVel.second);
		}
		if (abs(angAccTest.first) > maxThetaAcc)
		{
			maxThetaAcc = abs(angAccTest.first);
		}
		if (abs(angAccTest.second) > maxPhiAcc)
		{
			maxPhiAcc = abs(angAccTest.second);
		}
	}

	outfile.close();
	cout << file_name + " completed, " << maxThetaVel << ", " << maxPhiVel << ", " << maxThetaAcc << ", " << maxPhiAcc << endl;
}

void TaskSpaceTrajectoryCalc::generateCurvedTrajectory(string file_name, double startTheta, double startPhi, double x, double y, double distance, int type, double maxV, double maxA)
{
	pair<double, double> startXY = armKinematics::angToXY(startTheta, startPhi);
	double maxThetaAcc = 0;
	double maxThetaVel = 0;
	double maxPhiAcc = 0;
	double maxPhiVel = 0;
	
	bool phiPositive = (startPhi >= 0);
	//startXY.first += 0.05;

	if (sqrt(x * x + y * y) > shoulderArmLength_ + elbowArmLength_)
	{
		return;
	}

	if (x == startXY.first && y == startXY.second)
	{
		return;
	}

	TrajectoryCalc trajectoryCalc{ maxV, maxA, 0, 0, 0, 0 };
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

	double dt = 0.001;

	for (double i = 0; i < totalTime; i += dt)
	{
		tuple<double, double, double> profile = trajectoryCalc.getProfile(i); //a, v, p
		
		double angle = getSlope(type, wantedX);
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

		outfile << i << ", " << angles.first << ", " << angles.second << ", " << angVel.first << ", " << angVel.second << ", " << angAccTest.first << ", " << angAccTest.second <</* ", " << testPos.first << ", " << testPos.second << ", " << wantedX << ", " << wantedY << */endl;
		if (abs(angVel.first) > maxThetaVel) 
		{
			maxThetaVel = abs(angVel.first);
		}
		if (abs(angVel.second) > maxPhiVel)
		{
			maxPhiVel = abs(angVel.second);
		}
		if (abs(angAccTest.first) > maxThetaAcc)
		{
			maxThetaAcc = abs(angAccTest.first);
		}
		if (abs(angAccTest.second) > maxPhiAcc)
		{
			maxPhiAcc = abs(angAccTest.second);
		}

		//if((int)(i*100000.0) % (int)(dt * 10.0 * 100000.0) == 0)
		//{
		//	outfile << i << ", " << angles.first << ", " << angles.second << ", " << angVel.first << ", " << angVel.second << ", " << angAccTest.first << ", " << angAccTest.second << /*", " << testPos.first << ", " << testPos.second << ", " << wantedX << ", " << wantedY << */endl;
		//}
		

		wantedX += xVel * dt;
		//wantedY += yVel * dt;
		wantedY = calcY(type, wantedX);
	}

	outfile.close();
	cout << file_name + " completed, " <<  maxThetaVel << ", " << maxPhiVel << ", " << maxThetaAcc << ", " << maxPhiAcc << endl;
}

void TaskSpaceTrajectoryCalc::generateSpecialTrajectory(string file_name, double startTheta, double startPhi, double x, double y, double distance, int type, double maxV, double maxA)
{
	pair<double, double> startXY = armKinematics::angToXY(startTheta, startPhi);
	double maxThetaAcc = 0;
	double maxThetaVel = 0;
	double maxPhiAcc = 0;
	double maxPhiVel = 0;

	bool phiPositive = (startPhi >= 0);
	//startXY.first += 0.05;

	if (sqrt(x * x + y * y) > shoulderArmLength_ + elbowArmLength_)
	{
		return;
	}

	if (x == startXY.first && y == startXY.second)
	{
		return;
	}

	TrajectoryCalc trajectoryCalc{ maxV, maxA, 0, 0, 0, 0 };
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

	double dt = 0.001;

	for (double i = 0; i < totalTime; i += dt)
	{
		tuple<double, double, double> profile = trajectoryCalc.getProfile(i); //a, v, p

		pair<double, double> tangentInterceptXY = getTangent(type);
		if(wantedX >= 0 && wantedX < tangentInterceptXY.first)
		{
			//Circle thing
			angles.first = 0;
			//angles.second set by iteration
			double radialVel = (get<1>(profile) / (TwoJointArmConstants::FOREARM_LENGTH + TwoJointArmConstants::EE_LENGTH)) * 180.0 / M_PI;
			if (type > 0)
			{
				radialVel *= -1;
			}
			
			angVel.first = 0.0;
			angVel.second = radialVel;
			
		}
		else if(wantedX >= tangentInterceptXY.first)
		{
			double angle = getSlope(type, wantedX);

			xVel = get<1>(profile) * cos(angle);
			yVel = get<1>(profile) * sin(angle);

			if (type < 0)
			{
				xVel *= -1;
				yVel *= -1;
			}

			angles = armKinematics::xyToAng(wantedX, wantedY, phiPositive);

			angVel = armKinematics::linVelToAngVel(xVel, yVel, angles.first, angles.second);

		}
		else
		{
			//cube intake opposite side
			double angle = -getSlope(01, -wantedX);

			xVel = get<1>(profile) * cos(angle);
			yVel = get<1>(profile) * sin(angle);

			if (type < 0)
			{
				xVel *= -1;
				yVel *= -1;
			}

			double radius = sqrt(wantedX * wantedX + wantedY * wantedY);
			if(radius < 0.3175)
			{
				wantedX = 0;
				wantedY = -0.3175;
				angles.first = 0;
				angles.second = 180;
			}
			else
			{
				angles = armKinematics::xyToAng(wantedX, wantedY, !phiPositive);
			}

			if(angles.second < 0)
			{
				angles.second += 360;
			}

			angVel = armKinematics::linVelToAngVel(xVel, yVel, angles.first, angles.second);

		}

		angAccTest = { (angVel.first - prevAngVel.first) / dt, (angVel.second - prevAngVel.second) / dt };
		prevAngVel = angVel;

		testPos = { testPos.first + angVel.first * dt, testPos.second + angVel.second * dt };

		outfile << i << ", " << angles.first << ", " << angles.second << ", " << angVel.first << ", " << angVel.second << ", " << angAccTest.first << ", " << angAccTest.second <</* ", " << testPos.first << ", " << testPos.second << ", " << wantedX << ", " << wantedY << */endl;
		if (abs(angVel.first) > maxThetaVel)
		{
			maxThetaVel = abs(angVel.first);
		}
		if (abs(angVel.second) > maxPhiVel)
		{
			maxPhiVel = abs(angVel.second);
		}
		if (abs(angAccTest.first) > maxThetaAcc)
		{
			maxThetaAcc = abs(angAccTest.first);
		}
		if (abs(angAccTest.second) > maxPhiAcc)
		{
			maxPhiAcc = abs(angAccTest.second);
		}

		

		if(wantedX >= 0 && wantedX < tangentInterceptXY.first)
		{
			angles.second += angVel.second * dt;

			pair<double, double> xy = armKinematics::angToXY(angles.first, angles.second);
			//cout << xy.first << ", " << xy.second << ", " << angles.first << ", " << angles.second << endl;
			wantedX = xy.first;
			wantedY = xy.second;
		}
		else if(wantedX < 0)
		{
			wantedX += xVel * dt;
			//wantedY += yVel * dt;
			wantedY = calcY(01, wantedX);
		}
		else
		{
			wantedX += xVel * dt;
			//wantedY += yVel * dt;
			wantedY = calcY(type, wantedX);
		}
	}

	outfile.close();
	cout << file_name + " completed, " << maxThetaVel << ", " << maxPhiVel << ", " << maxThetaAcc << ", " << maxPhiAcc << endl;
}

void TaskSpaceTrajectoryCalc::generateAutoSpecialTrajectory(string file_name, double startTheta, double startPhi, double x, double y, double distance, int type, double maxV, double maxA)
{
	pair<double, double> startXY = armKinematics::angToXY(startTheta, startPhi);
	double maxThetaAcc = 0;
	double maxThetaVel = 0;
	double maxPhiAcc = 0;
	double maxPhiVel = 0;

	bool phiPositive = (startPhi >= 0);
	//startXY.first += 0.05;

	if (sqrt(x * x + y * y) > shoulderArmLength_ + elbowArmLength_)
	{
		return;
	}

	if (x == startXY.first && y == startXY.second)
	{
		return;
	}

	TrajectoryCalc trajectoryCalc{ maxV, maxA, 0, 0, 0, 0 };
	trajectoryCalc.generateTrajectory(0, distance, 0);

	double totalTime = trajectoryCalc.getTotalTime();
	double xVel = 0;
	double yVel = 0;
	double xAcc = 0;
	double yAcc = 0;
	double wantedX = startXY.first;
	double wantedY = startXY.second;
	pair<double, double> angVel;
	pair<double, double> angAcc;

	pair<double, double> angAccTest;
	pair<double, double> prevAngVel{ 0, 0 };

	pair<double, double> angles;
	if(type > 0)
	{
		angles.first = -18.5;
		angles.second = 179.9999;
	}
	else
	{
		angles.first = startTheta;
		angles.second = startPhi;
	}
	pair<double, double> testPos{ startTheta, startPhi };
	ofstream outfile(file_name);

	double dt = 0.001;

	for (double i = 0; i < totalTime; i += dt)
	{
		tuple<double, double, double> profile = trajectoryCalc.getProfile(i); //a, v, p

		//pair<double, double> tangentInterceptXY = { 0.35105,-0.17901 };
		pair<double, double> tangentInterceptXY = getTangent(type);
		if (wantedX >= 0 && wantedX < tangentInterceptXY.first)
		{
			//Circle thing
			angles.first = -18.5;
			//angles.second set by iteration
			double radialVel = (get<1>(profile) / (TwoJointArmConstants::FOREARM_LENGTH + TwoJointArmConstants::EE_LENGTH)) * 180.0 / M_PI;
			if (type > 0)
			{
				radialVel *= -1;
			}

			angVel.first = 0.0;
			angVel.second = radialVel;

		}
		else if (wantedX >= tangentInterceptXY.first)
		{
			double angle = getSlope(type, wantedX);

			xVel = get<1>(profile) * cos(angle);
			yVel = get<1>(profile) * sin(angle);

			if (type < 0)
			{
				xVel *= -1;
				yVel *= -1;
			}

			angles = armKinematics::xyToAng(wantedX, wantedY, phiPositive);

			angVel = armKinematics::linVelToAngVel(xVel, yVel, angles.first, angles.second);

			//cout << wantedX << ", " << wantedY << ", " << angles.first << ", " << angles.second << endl;
		}

		angAccTest = { (angVel.first - prevAngVel.first) / dt, (angVel.second - prevAngVel.second) / dt };
		prevAngVel = angVel;

		testPos = { testPos.first + angVel.first * dt, testPos.second + angVel.second * dt };

		outfile << i << ", " << angles.first << ", " << angles.second << ", " << angVel.first << ", " << angVel.second << ", " << angAccTest.first << ", " << angAccTest.second <</* ", " << testPos.first << ", " << testPos.second << ", " << wantedX << ", " << wantedY << */endl;
		if (abs(angVel.first) > maxThetaVel)
		{
			maxThetaVel = abs(angVel.first);
		}
		if (abs(angVel.second) > maxPhiVel)
		{
			maxPhiVel = abs(angVel.second);
		}
		if (abs(angAccTest.first) > maxThetaAcc)
		{
			maxThetaAcc = abs(angAccTest.first);
		}
		if (abs(angAccTest.second) > maxPhiAcc)
		{
			maxPhiAcc = abs(angAccTest.second);
		}



		if (wantedX >= 0 && wantedX < tangentInterceptXY.first)
		{
			angles.second += angVel.second * dt;

			pair<double, double> xy = armKinematics::angToXY(angles.first, angles.second);
			//cout << xy.first << ", " << xy.second << ", " << angles.first << ", " << angles.second << endl;
			wantedX = xy.first;
			wantedY = xy.second;
		}
		else
		{
			//cout << wantedX << ", " << xVel << ", " << dt << endl;
			wantedX += xVel * dt;
			//wantedY += yVel * dt;
			wantedY = calcY(type, wantedX);
		}
	}

	outfile.close();
	cout << file_name + " completed, " << maxThetaVel << ", " << maxPhiVel << ", " << maxThetaAcc << ", " << maxPhiAcc << endl;
}

double TaskSpaceTrajectoryCalc::getSlope(int type, double x)
{
	//double piecewiseSlope = atan2(0.717582878724, 1.0);
	if (x < 0.5 && type != 9 && type != -9 && type != 19 && type != -19 && type != 29 && type != -29 && type != 49 && type != -49 && type != 59 && type != -59 && type != 69 && type != -69 && type != 79 && type != -79 && type != 89 && type != -89 && type != 01 && type != -01 && type != 56 && type != -56 && type != 07 && type != -07 && type != 8 && type != -8)
	{
		//return atan2(15.6 * 3 * (x - 0.13813) * (x - 0.13813), 1.0); 11, 169
		//return atan2(3.87122 * 4.0 * (x - 0.01308) * (x - 0.01308) * (x - 0.01308), 1.0);//1, 1
		return atan2(3.48 * 4.0 * x * x * x, 1.0); //0, 180
		//return piecewiseSlope;
	}
	/*else if (x < 0.10074 && type == 29 || type == -29 || type == -49 || type == 49 || type == 59 || type == -59 || type == 69 || type == -69 || type == 79 || type == -79)
	{
		return atan2(x + 743.0 / 3125.0, sqrt((5777007.0 / 3125000.0) - (x + (743.0 / 3125.0)) * (x + (743.0 / 3125.0))));
	}*/
	else
	{
		switch (type)
		{
		case 01:
		{
			//return atan2(0.150216 * 2.0 * (x - 0.01308), 1.0);
			//return atan2(0.143776 * 2 * x, 1.0); //Original and working
			return atan2(0.26643 * 2 * x, 1.0); //New for bungee
		}
		case -01:
		{
			//return atan2(0.150216 * 2.0 * (x - 0.01308), 1.0);
			//return atan2(0.143776 * 2 * x, 1.0); //Original and working
			return atan2(0.26643 * 2 * x, 1.0); //New for bungee
		}
		case 02:
		{
			//return atan2(-1.3713 * 2.0 * (x - 1.14135), 1.0); //GOOD AND NOT PAINFUL
			return atan2(-1.58878 * 2.0 * (x - 1.1607), 1.0); //1, 1
		}
		case -02:
		{
			//return atan2(-18.921 * 4.0 * (x - 0.80425) * (x - 0.80425) * (x - 0.80425), 1.0);
			//return atan2(-3.5341 * 2.0 * (x - 0.9), 1.0); //GOOD AND NOT PAINFUL
			//return atan2(0.531703 * 4 * cos(4 * (x - 0.18556 - (pi / 8.0))), 1.0);
			return atan2(-3.09374 * 2.0 * (x - 1), 1.0); //1, 
		}
		//case 03:
		//{
		//	return atan2(-1.2937 * 2.0 * (x - 1.14708), 1.0); //GOOD AND NOT PAINFUL?
		//}
		//case -03:
		//{
		//	return atan2(-1.2937 * 2.0 * (x - 1.14708), 1.0); //GOOD AND NOT PAINFUL?
		//}
		case 04:
		{
			//return atan2(-0.9897 * 2.0 * (x - 1.44532), 1.0); //GOOD AND NOT PAINFUL
			return atan2(-0.948 * 2.0 * (x - 1.54007), 1.0);//1, 1
		}
		case -04:
		{
			//return atan2(-0.9897 * 2.0 * (x - 1.44532), 1.0); //GOOD AND NOT PAINFUL
			return atan2(-0.948 * 2.0 * (x - 1.54007), 1.0);//1, 1
		}
		case 05:
		{
			//return atan2(1.0, 10.0 * (x - (1739.0 / 5000.0)));
			//return atan2(-1.5993 * 2.0 * (x - 0.98065), 1.0); //GOOD AND NOT PAINFUL
			return atan2(-1.6716 * 2.0 * (x - 1.02963), 1.0);//1, 1
		}
		case -05:
		{
			//return atan2(1.0, 10.0 * (x - (1739.0 / 5000.0)));
			//return atan2(-1.5993 * 2.0 * (x - 0.98065), 1.0); //GOOD AND NOT PAINFUL
			return atan2(-1.6716 * 2.0 * (x - 1.02963), 1.0);//1, 1
		}
		case 06:
		{
			//return atan2(-1.0481 * 2.0 * (x - 1.32466), 1.0); //GOOD AND NOT PAINFUL
			return atan2(-0.87557 * 2.0 * (x - 1.49726), 1.0);//1, 1
		}
		case -06:
		{
			//return atan2(-1.0481 * 2.0 * (x - 1.32466), 1.0); //GOOD AND NOT PAINFUL
			return atan2(-0.87557 * 2.0 * (x - 1.49726), 1.0);//1, 1
		}
		case 07:
		{
			return atan2(-1.252 * 6 * x * x * x * x * x, 1.0);
			//return atan2(0.2979763 * 2.0 * (x), 1.0);
		}
		case -07:
		{
			return atan2(-1.252 * 6 * x * x * x * x * x, 1.0);
			//return atan2(0.2979763 * 2.0 * (x), 1.0);
		}
		case 8:
		{
			return atan2(1.50173 * 2.0 * x, 1.0);
		}
		case -8:
		{
			return atan2(1.50173 * 2.0 * x, 1.0);
		}
		case 9:
		{
			//return atan2(-3.553 * 2 * (x - 0.3526), 1.0);//GOOD AND NOT PAINFUL
			//return atan2(-1.9722 * 2.0 * (x - 0.32), 1.0);
			return atan2(1.12377 * 2.0 * x, 1.0);
		}
		case -9:
		{
			//return atan2(-3.553 * 2 * (x - 0.3526), 1.0);//GOOD AND NOT PAINFUL
			//return atan2(-1.9722 * 2.0 * (x - 0.32), 1.0);
			return atan2(1.12377 * 2.0 * x, 1.0);
		}
		case 29:
		{
			//return atan2(-1.17851 * 2.0 * (x - 1.1607), 1.0);
			return atan2(-1.84870016 * 2.0 * (x - 1.1607), 1.0);
		}
		case -29:
		{
			//return atan2(-1.17851 * 2.0 * (x - 1.1607), 1.0);
			return atan2(-1.84870016 * 2.0 * (x - 1.1607), 1.0);
		}
		case 49:
		{
			//return atan2(-0.78125 * 2.0 * (x - 1.54007), 1.0);
			return atan2(-1.0261841144 * 2.0 * (x - 1.54007), 1.0);
		}
		case -49:
		{
			//return atan2(-0.78125 * 2.0 * (x - 1.54007), 1.0);
			return atan2(-1.0261841144 * 2.0 * (x - 1.54007), 1.0);
		}
		case 59:
		{
			//return atan2(-1.18987 * 2.0 * (x - 1.02963), 1.0);
			return atan2(-1.745968432 * 2.0 * (x - 1.02963), 1.0);
		}
		case -59:
		{
			//return atan2(-1.18987 * 2.0 * (x - 1.02963), 1.0);
			return atan2(-1.745968432 * 2.0 * (x - 1.02963), 1.0);
		}
		case 69:
		{
			//return atan2(-0.88569 * 2.0 * (x - 1.38), 1.0);
			return atan2(-1.208114893 * 2.0 * (x - 1.38), 1.0);
		}
		case -69:
		{
			//return atan2(-0.88569 * 2.0 * (x - 1.38), 1.0);
			return atan2(-1.208114893 * 2.0 * (x - 1.38), 1.0);
		}
		case 79:
		{
			//return atan2(-0.396221 * 2.0 * (x - 1.10306), 1.0);
			return atan2(-0.397722048 * 2.0 * (x - 1.10306), 1.0);
		}
		case -79:
		{
			//return atan2(-0.396221 * 2.0 * (x - 1.10306), 1.0);
			return atan2(-0.397722048 * 2.0 * (x - 1.10306), 1.0);
		}
		case 56:
		{
			//return atan2(8217.0, 100000.0 * (x - (91.0 / 100.0)));
			return atan2(-3.1754 * 2.0 * (x - 1.38), 1.0); //GOOD AND NOT PAINFUL
		}
		case -56:
		{
			//return atan2(8217.0, 100000.0 * (x - (91.0 / 100.0)));
			return atan2(-3.1754 * 2.0 * (x - 1.38), 1.0); //GOOD AND NOT PAINFUL
		}
		case 34:
		{
			return atan2(-1.9232992073 * 2.0 * (x - 1.54007), 1.0);
		}
		case -34:
		{
			return atan2(-1.9232992073 * 2.0 * (x - 1.54007), 1.0);
		}
		case 23:
		{
			return atan2(-8.58337692778 * 2.0 * (x - 1.1607), 1.0);
		}
		case -23:
		{
			return atan2(-8.58337692778 * 2.0 * (x - 1.1607), 1.0);
		}
		case 53:
		{
			return atan2(-19.2906005089 * 2.0 * (x - 1.02963), 1.0);
		}
		case -53:
		{
			return atan2(-19.2906005089 * 2.0 * (x - 1.02963), 1.0);
		}
		case 63:
		{
			return atan2(-2.7267422476 * 2.0 * (x - 1.38), 1.0);
		}
		case -63:
		{
			return atan2(-2.7267422476 * 2.0 * (x - 1.38), 1.0);
		}
		default:
		{
			return 0;
		}
		}
	}
}

double TaskSpaceTrajectoryCalc::calcY(int type, double x)
{
	//double piecewiseSlope = atan2(0.717582878724, 1.0);
	if (x < 0.5 && type != 9 && type != -9 && type != 19 && type != -19 && type != 29 && type != -29 && type != 49 && type != -49 && type != 59 && type != -59 && type != 69 && type != -69 && type != 79 && type != -79 && type != 89 && type != -89 && type != 01 && type != -01 && type != 56 && type != -56 && type != 07 && type != -07 && type != 8 && type != -8)
	{
		//return atan2(15.6 * 3 * (x - 0.13813) * (x - 0.13813), 1.0); 11, 169
		//return atan2(3.87122 * 4.0 * (x - 0.01308) * (x - 0.01308) * (x - 0.01308), 1.0);//1, 1
		return 3.48 * x * x * x * x - 0.3175; //0, 180
		//return piecewiseSlope;
	}
	else if (x < 0.10074 && (type == 29 || type == -29 || type == -49 || type == 49 || type == 59 || type == -59 || type == 69 || type == -69 || type == 79 || type == -79))
	{
		return -sqrt(1.13806244 - (x + 0.23776) * (x + 0.23776)) + 0.71058;
	}
	else
	{
		switch (type)
		{
		case 01:
		{
			//return atan2(0.150216 * 2.0 * (x - 0.01308), 1.0);
			//return 0.143776 * x * x - 0.3175; //Original and working
			return 0.26643 * x * x - 0.3175; //New for bungee
		}
		case -01:
		{
			//return atan2(0.150216 * 2.0 * (x - 0.01308), 1.0);
			//return 0.143776 * x * x - 0.3175; //Original and working
			return 0.26643 * x * x - 0.3175; //New for bungee
		}
		case 02:
		{
			//return atan2(-1.3713 * 2.0 * (x - 1.14135), 1.0); //GOOD AND NOT PAINFUL
			return -1.58878 * (x - 1.1607) * (x - 1.1607) + 0.59354; //1, 1
		}
		case -02:
		{
			//return atan2(-18.921 * 4.0 * (x - 0.80425) * (x - 0.80425) * (x - 0.80425), 1.0);
			//return atan2(-3.5341 * 2.0 * (x - 0.9), 1.0); //GOOD AND NOT PAINFUL
			//return atan2(0.531703 * 4 * cos(4 * (x - 0.18556 - (pi / 8.0))), 1.0);
			return -3.09374 * (x - 1) * (x - 1) + 0.673435; //1, 
		}
		//case 03:
		//{
		//	return atan2(-1.2937 * 2.0 * (x - 1.14708), 1.0); //GOOD AND NOT PAINFUL?
		//}
		//case -03:
		//{
		//	return atan2(-1.2937 * 2.0 * (x - 1.14708), 1.0); //GOOD AND NOT PAINFUL?
		//}
		case 04:
		{
			//return atan2(-0.9897 * 2.0 * (x - 1.44532), 1.0); //GOOD AND NOT PAINFUL
			return -0.948 * (x - 1.54007) * (x - 1.54007) + 0.92549;//1, 1
		}
		case -04:
		{
			//return atan2(-0.9897 * 2.0 * (x - 1.44532), 1.0); //GOOD AND NOT PAINFUL
			return -0.948 * (x - 1.54007) * (x - 1.54007) + 0.92549;//1, 1
		}
		case 05:
		{
			//return atan2(1.0, 10.0 * (x - (1739.0 / 5000.0)));
			//return atan2(-1.5993 * 2.0 * (x - 0.98065), 1.0); //GOOD AND NOT PAINFUL
			return -1.6716 * (x - 1.02963) * (x - 1.02963) + 0.36889;//1, 1
		}
		case -05:
		{
			//return atan2(1.0, 10.0 * (x - (1739.0 / 5000.0)));
			//return atan2(-1.5993 * 2.0 * (x - 0.98065), 1.0); //GOOD AND NOT PAINFUL
			return -1.6716 * (x - 1.02963) * (x - 1.02963) + 0.36889;//1, 1
		}
		case 06:
		{
			//return atan2(-1.0481 * 2.0 * (x - 1.32466), 1.0); //GOOD AND NOT PAINFUL
			return -0.87557 * (x - 1.49726) * (x - 1.49726) + 0.77077;//1, 1
		}
		case -06:
		{
			//return atan2(-1.0481 * 2.0 * (x - 1.32466), 1.0); //GOOD AND NOT PAINFUL
			return -0.87557 * (x - 1.49726) * (x - 1.49726) + 0.77077;//1, 1
		}
		case 07:
		{
			return -1.252 * x * x * x * x * x * x - 0.3175;
			//return 0.2979763 * (x) * (x) - 0.3175;
		}
		case -07:
		{
			return -1.252 * x * x * x * x * x * x - 0.3175;
			//return 0.2979763 * (x) * (x) - 0.3175;
		}
		case 8:
		{
			return 1.50173 * x * x - 0.3175;
		}
		case -8:
		{
			return 1.50173 * x * x - 0.3175;
		}
		case 9:
		{
			//return atan2(-3.553 * 2 * (x - 0.3526), 1.0);//GOOD AND NOT PAINFUL
			//return atan2(-1.9722 * 2.0 * (x - 0.32), 1.0);
			//return -1.252 * x * x * x * x * x * x - 0.3175;
			return 1.12377 * x * x - 0.3175;
		}
		case -9:
		{
			//return atan2(-3.553 * 2 * (x - 0.3526), 1.0);//GOOD AND NOT PAINFUL
			//return atan2(-1.9722 * 2.0 * (x - 0.32), 1.0);
			//return -1.252 * x * x * x * x * x * x - 0.3175;
			return 1.12377 * x * x - 0.3175;
		}
		case 29:
		{
			//return -1.17851 * (x - 1.1607) * (x - 1.1607) + 0.59354;
			return -1.84870016 * (x - 1.1607) * (x - 1.1607) + 0.59354;
		}
		case -29:
		{
			//return -1.17851 * (x - 1.1607) * (x - 1.1607) + 0.59354;
			return -1.84870016 * (x - 1.1607) * (x - 1.1607) + 0.59354;
		}
		case 49:
		{
			//return -0.78125 * (x - 1.54007) * (x - 1.54007) + 0.92549;
			return -1.0261841144 * (x - 1.54007) * (x - 1.54007) + 0.92549;
		}
		case -49:
		{
			//return -0.78125 * (x - 1.54007) * (x - 1.54007) + 0.92549;
			return -1.0261841144 * (x - 1.54007) * (x - 1.54007) + 0.92549;
		}
		case 59:
		{
			//return -1.18987 * (x - 1.02963) * (x - 1.02963) + 0.36889;
			return -1.745968432 * (x - 1.02963) * (x - 1.02963) + 0.36889;
		}
		case -59:
		{
			//return -1.18987 * (x - 1.02963) * (x - 1.02963) + 0.36889;
			return - 1.745968432 * (x - 1.02963) * (x - 1.02963) + 0.36889;
		}
		case 69:
		{
			//return -0.88569 * (x - 1.38) * (x - 1.38) + 0.7587;
			return -1.208114893 * (x - 1.38) * (x - 1.38) + 0.7587;
		}
		case -69:
		{
			//return -0.88569 * (x - 1.38) * (x - 1.38) + 0.7587;
			return -1.208114893 * (x - 1.38) * (x - 1.38) + 0.7587;
		}
		case 79:
		{
			//return -0.396221 * (x - 1.10306) * (x - 1.10306) + 0.04506;
			return -0.397722048 * (x - 1.10306) * (x - 1.10306) + 0.04506;
		}
		case -79:
		{
			//return -0.396221 * (x - 1.10306) * (x - 1.10306) + 0.04506;
			return -0.397722048 * (x - 1.10306) * (x - 1.10306) + 0.04506;
		}
		case 56:
		{
			//return atan2(8217.0, 100000.0 * (x - (91.0 / 100.0)));
			return -3.1754 * (x - 1.38) * (x - 1.38) + 0.7587; //GOOD AND NOT PAINFUL
		}
		case -56:
		{
			//return atan2(8217.0, 100000.0 * (x - (91.0 / 100.0)));
			return -3.1754 * (x - 1.38) * (x - 1.38) + 0.7587; //GOOD AND NOT PAINFUL
		}
		case 34:
		{
			return -1.9232992073 * (x - 1.54007) * (x - 1.54007) + 0.92549;
		}
		case -34:
		{
			return -1.9232992073 * (x - 1.54007) * (x - 1.54007) + 0.92549;
		}
		case 23:
		{
			return -8.58337692778 * (x - 1.1607) * (x - 1.1607) + 0.59354;
		}
		case -23:
		{
			return -8.58337692778 * (x - 1.1607) * (x - 1.1607) + 0.59354;
		}
		case 53:
		{
			return -19.2906005089 * (x - 1.02963) * (x - 1.02963) + 0.36889;
		}
		case -53:
		{
			return -19.2906005089 * (x - 1.02963) * (x - 1.02963) + 0.36889;
		}
		case 63:
		{
			return -2.7267422476 * (x - 1.38) * (x - 1.38) + 0.7587;
		}
		case -63:
		{
			return -2.7267422476 * (x - 1.38) * (x-1.38) + 0.7587;
		}
		default:
		{
			return 0;
		}
		}
	}
}

pair<double, double> TaskSpaceTrajectoryCalc::getTangent(int type)
{
	switch(type)
	{
	case 34:
	{
		return { 0.971896, 0.304607 };
	}
	case -34:
	{
		return { 0.971896, 0.304607 };
	}
	case 23:
	{
		return { 1.0027657, 0.379443 };
	}
	case -23:
	{
		return { 1.0027657, 0.379443 };
	}
	case 53:
	{
		return { 0.972707, 0.306384 };
	}
	case -53:
	{
		return { 0.972707, 0.306384 };
	}
	case 63:
	{
		return { 0.974043, 0.30933 };
	}
	case -63:
	{
		return { 0.974043, 0.30933 };
	}
	case 29:
	{
		return { 0.688128, 0.1806789 };
	}
	case -29:
	{
		return { 0.688128, 0.1806789 };
	}
	case 49:
	{
		return { 0.688252, 0.180897 };
	}
	case -49:
	{
		return { 0.688252, 0.180897 };
	}
	case 59:
	{
		return { 0.629643, 0.089553 };
	}
	case -59:
	{
		return { 0.629643, 0.089553 };
	}
	case 69:
	{
		return { 0.680405, 0.16741 };
	}
	case -69:
	{
		return { 0.680405, 0.16741 };
	}
	case 79:
	{
		return { 0.324124, -0.196256 };
	}
	case -79:
	{
		return { 0.324124, -0.196256 };
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