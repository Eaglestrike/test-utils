#pragma once

#include <iostream>
#include <math.h>
#include <string.h>
#include "Constants.h"
#include "Helpers.h"
#include "TrajectoryCalc.h"
#include "TwoJointArmProfiles.h"
#include "armKinematics.h"

class TwoJointArm
{
public:
    TwoJointArm();
    enum State
    {
        HOLDING_POS,
        FOLLOWING_TASK_SPACE_PROFILE,
        FOLLOWING_JOINT_SPACE_PROFILE,
        HOMING,
        STOPPED,
        MANUAL
    };
    State getState();

    void perioidc(double time);
    void setPosTo(TwoJointArmProfiles::Positions setPosition);
    void setBrakes(bool engaged);
    void stop();
    void followTaskSpaceProfile(double time);//COULDO combine into one function to calculate
    void followJointSpaceProfile(double time);
    void home(double time);

    double calcShoulderVolts(double vel, double acc, double theta, double phi, bool hasCone);
    double calcElbowVolts(double vel, double acc, double theta, double phi, double hasCone);

    double calcShoulderRotInertia(double phi, bool hasCone);
    double calcElbowRotInertia(bool hasCone);

    double calcShoulderGravityTorque(double theta, double phi, bool hasCone);
    double calcElbowGravityTorque(double theta, double phi, bool hasCone);

    double calcKT(double volts);
    double calcR(double volts);

    double getTheta();
    double getPhi();

private:
    double taskSpaceStartTime_, jsst_;

    TrajectoryCalc shoulderTraj_, elbowTraj_;

    TwoJointArmProfiles movementProfiles_;
    armKinematics armKinematics_;

    TwoJointArmProfiles::Positions position_, setPosition_;
    pair<TwoJointArmProfiles::Positions, TwoJointArmProfiles::Positions> key_;

    State state_;

    bool posUnknown_, brakesEngaged_;
    pair<bool, bool> homing_;
    double prevShoulderVolts_, prevElbowVolts_;

    double skV_ = 0;
    double skP_ = 0;
    double ekV_ = 0;
    double ekP_ = 0;

    double time_;
};