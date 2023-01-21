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
	for(double i = 0; i < totalTime; i += 0.0001)
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
		angAcc = armKinematics::linVelToAngVel(xAcc, yAcc, angles.first, angles.second); //TODO not correct

		angAccTest = { (angVel.first - prevAngVel.first) / 0.0001, (angVel.second - prevAngVel.second) / 0.0001 };
		prevAngVel = angVel;

		testPos = { testPos.first + angVel.first * 0.0001, testPos.second + angVel.second * 0.0001 };
		//cout << angVel.first << ", " << testPos.first << endl;

		/*outfile << i << ", " << angles.first << ", " << angles.second << ", " << angVel.first << ", " << angVel.second << ", " << angAcc.first << ", " << angAcc.second <<
		", " << wantedX << ", " << wantedY << ", " << xVel << ", " << yVel << ", " << xAcc << ", " << yAcc << ", " << angAccTest.first << ", " << angAccTest.second << 
		", " << testPos.first << ", " << testPos.second << endl;*/

		outfile << i << ", " << angles.first << ", " << angles.second << ", " << angVel.first << ", " << angVel.second << ", " << angAcc.first << ", " << angAcc.second << endl;
	}

	outfile.close();
	//cout << file_name << " complete" << endl;
}