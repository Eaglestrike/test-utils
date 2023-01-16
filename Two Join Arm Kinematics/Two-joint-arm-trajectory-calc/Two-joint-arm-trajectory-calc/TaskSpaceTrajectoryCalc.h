#pragma once
#include "armKinematics.h"
#include "TrajectoryCalc.h"
#include <string>

class TaskSpaceTrajectoryCalc
{
public:
	TaskSpaceTrajectoryCalc(double LA_MAX_V, double LA_MAX_A, double UA_MAX_V, double UA_MAX_A, double LA_LENGTH, double UA_LENGTH, double TASK_MAX_V, double TASK_MAX_A);
	void generateLinearTrajectory(string file_name, double startTheta, double startPhi, double x, double y);

private:
	double laMaxV_, laMaxA_, uaMaxV_, uaMaxA_, laLength_, uaLength_, taskMaxV_, taskMaxA_;

};

