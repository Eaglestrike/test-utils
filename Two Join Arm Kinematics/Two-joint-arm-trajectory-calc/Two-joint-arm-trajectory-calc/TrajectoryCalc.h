#pragma once
#include <math.h>
#include <iostream>
#include <tuple>

#include "Constants.h"

using namespace std;

class TrajectoryCalc
{
public:
    TrajectoryCalc(double maxV, double maxA, double kP, double kD, double kV, double kA);
    TrajectoryCalc(double maxV, double maxA, double kP, double kD, double kV, double kA, double kVI);

    void setKP(double kP);
    void setKD(double kD);
    void setKV(double kV);
    void setKA(double kA);
    void setKVI(double kVI);

    void generateTrajectory(double pos, double setPos, double vel);

    tuple<double, double, double> getProfile(double time);

    double calcPower(double pos, double vel, double time);

    void setPrintError(bool printError);

    double getTotalTime();

private:
    const double MAX_V, MAX_A;
    double kP_, kD_, kV_, kA_, kVI_;

    double /*prevAbsoluteError_,*/ setPos_, setVel_, initPos_, initVel_, startTime_;

    int direction_;
    double cruiseSpeed_, cruiseDist_, accelDist_, deccelDist_, cruiseTime_, accelTime_, deccelTime_;

    bool printError_ = false;
};