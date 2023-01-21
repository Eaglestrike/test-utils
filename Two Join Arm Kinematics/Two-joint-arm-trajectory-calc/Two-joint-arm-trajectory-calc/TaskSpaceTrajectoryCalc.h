#pragma once
#include "armKinematics.h"
#include "TrajectoryCalc.h"
#include <string>

class TaskSpaceTrajectoryCalc
{
public:
	TaskSpaceTrajectoryCalc(double SHOULDER_ARM_MAX_V, double SHOULDER_ARM_MAX_A, double ELBOW_ARM_MAX_V, double ELBOW_ARM_MAX_A, double SHOULDER_ARM_LENGTH, double ELBOW_ARM_LENGTH, double TASK_MAX_V, double TASK_MAX_A);
	void generateLinearTrajectory(string file_name, double startTheta, double startPhi, double x, double y);

private:
	double shoulderArmMaxV_, shoulderArmMaxA_, elbowArmMaxV_, elbowArmMaxA_, shoulderArmLength_, elbowArmLength_, taskMaxV_, taskMaxA_;

};

