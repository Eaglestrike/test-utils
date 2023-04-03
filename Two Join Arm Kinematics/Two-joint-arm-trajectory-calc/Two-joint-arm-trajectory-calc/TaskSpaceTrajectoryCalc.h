#pragma once
#include "armKinematics.h"
#include "TrajectoryCalc.h"
#include <string>
#include <map>

class TaskSpaceTrajectoryCalc
{
public:
	TaskSpaceTrajectoryCalc(double SHOULDER_ARM_MAX_V, double SHOULDER_ARM_MAX_A, double ELBOW_ARM_MAX_V, double ELBOW_ARM_MAX_A, double SHOULDER_ARM_LENGTH, double ELBOW_ARM_LENGTH/*, double TASK_MAX_V, double TASK_MAX_A*/);
	void generateLinearTrajectory(string file_name, double startTheta, double startPhi, double x, double y, double maxV, double maxA);
	void generateCurvedTrajectory(string file_name, double startTheta, double startPhi, double x, double y, double distance, int type, double maxV, double maxA);
	void generateSpecialTrajectory(string file_name, double startTheta, double startPhi, double x, double y, double distance, int type, double maxV, double maxA);
	void generateAutoSpecialTrajectory(string file_name, double startTheta, double startPhi, double x, double y, double distance, int type, double maxV, double maxA);
	double getSlope(int type, double x);
	double calcY(int type, double x);
	pair<double, double> getTangent(int type);

	void generateLinearTrajectoryOptimized(string file_name, double startTheta, double startPhi, double x, double y);

	//map<double, tuple<pair<double, double>, pair<double, double>, pair<double, double>>> generateLinMaxAccelTrajectory(pair<double, double> startXY, pair<double, double> startAng, pair<double, double> startAngVel, pair<double, double> endXY);

private:
	double shoulderArmMaxV_, shoulderArmMaxA_, elbowArmMaxV_, elbowArmMaxA_, shoulderArmLength_, elbowArmLength_/*, taskMaxV_, taskMaxA_*/;

};

