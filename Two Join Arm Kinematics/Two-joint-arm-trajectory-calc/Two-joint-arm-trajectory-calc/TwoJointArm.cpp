#include "TwoJointArm.h"

TwoJointArm::TwoJointArm() : shoulderTraj_(TwoJointArmConstants::SHOULDER_ARM_MAX_VEL, TwoJointArmConstants::SHOULDER_ARM_MAX_ACC, 0, 0, 0, 0), elbowTraj_(TwoJointArmConstants::ELBOW_ARM_MAX_VEL, TwoJointArmConstants::ELBOW_ARM_MAX_ACC, 0, 0, 0, 0)
{

    movementProfiles_.readProfiles();
    state_ = HOLDING_POS;
    position_ = TwoJointArmProfiles::STOWED;
    setPosition_ = TwoJointArmProfiles::STOWED;
    posUnknown_ = true;
    homing_ = { false, false };
    setBrakes(true);
    taskSpaceStartTime_ = 0;

    prevShoulderVolts_ = 0;
    prevElbowVolts_ = 0;

    time_ = 0;
}

TwoJointArm::State TwoJointArm::getState()
{
    return state_;
}

void TwoJointArm::perioidc(double time)
{
    time_ = time;
    switch (state_)
    {
    case HOLDING_POS:
    {
        setBrakes(true);
        stop();
        if (position_ != setPosition_)
        {
            setPosTo(setPosition_);
        }

        break;
    }
    case FOLLOWING_TASK_SPACE_PROFILE:
    {
        setBrakes(false);
        followTaskSpaceProfile(time - taskSpaceStartTime_);
        break;
    }
    case FOLLOWING_JOINT_SPACE_PROFILE:
    {
        setBrakes(false);
        followJointSpaceProfile(time - jsst_);
        break;
    }
    case HOMING:
    {
        setBrakes(false);
        home(time_);
        break;
    }
    case STOPPED:
    {
        setBrakes(true);
        posUnknown_ = true;
        break;
    }
    case MANUAL:
    {
        setBrakes(false);
        //TODO manual yeah
        break;
    }
    }
}

void TwoJointArm::setPosTo(TwoJointArmProfiles::Positions setPosition)
{
    if (state_ == STOPPED)
    {
        state_ = HOMING;
    }

    if (state_ == HOMING)
    {
        setPosition_ = setPosition;
        return;
    }

    if (position_ == setPosition)
    {
        return;
    }

    if(state_ == FOLLOWING_TASK_SPACE_PROFILE)
    {
        return;
    }

    setPosition_ = setPosition;

    //TODO figure out if arm position matches position before doing anything
    key_ = { position_, setPosition_ };
    state_ = FOLLOWING_TASK_SPACE_PROFILE;
    taskSpaceStartTime_ = time_; // timer_.GetFPGATimestamp().value();
}

void TwoJointArm::setBrakes(bool engaged)
{
    brakesEngaged_ = engaged;
}
void TwoJointArm::stop()
{
}

void TwoJointArm::followTaskSpaceProfile(double time)
{
    tuple<double, double, double> thetaProfile = movementProfiles_.getThetaProfile(key_, time);
    tuple<double, double, double> phiProfile = movementProfiles_.getPhiProfile(key_, time);

    double wantedTheta = get<0>(thetaProfile);
    double wantedPhi = get<0>(phiProfile);
    double thetaVel = get<1>(thetaProfile);
    double phiVel = get<1>(phiProfile);
    double thetaAcc = get<2>(thetaProfile);
    double phiAcc = get<2>(phiProfile);

    double theta = wantedTheta; // getTheta();
    double phi = wantedPhi; // getPhi();
    double shoulderVel = thetaVel; // shoulderMaster_.GetSelectedSensorVelocity()* (10.0 / 2048.0) * 360.0 * TwoJointArmConstants::MOTOR_TO_SHOULDER_RATIO;
    double elbowVel = phiVel; // elbowMaster_.GetSelectedSensorVelocity()* (10.0 / 2048.0) * 360.0 * TwoJointArmConstants::MOTOR_TO_ELBOW_RATIO - shoulderVel * TwoJointArmConstants::SHOULDER_TO_ELBOW_RATIO;

    if (thetaVel == 0 && phiVel == 0 && thetaAcc == 0 && phiAcc == 0)
    {
        position_ = setPosition_;
        if (abs(theta - wantedTheta) > 20 || abs(phi - wantedPhi) > 20) //TODO get values
        {
            cout << "stopped" << endl;
            state_ = STOPPED;
        }
        else if (abs(theta - wantedTheta) > 10 || abs(phi - wantedPhi) > 10)
        {
            cout << "got close" << endl;
            shoulderTraj_.generateTrajectory(theta, wantedTheta, thetaVel);
            elbowTraj_.generateTrajectory(phi, wantedPhi, phiVel);
            jsst_ = time_;
            state_ = FOLLOWING_JOINT_SPACE_PROFILE;
        }
        else
        {
            cout << "done moving" << endl;
            state_ = HOLDING_POS;
        }
    }

    double thetaVolts = calcShoulderVolts(thetaVel, thetaAcc, theta, phi, false); //TODO check if has cone is viable or necesary
    double phiVolts = calcElbowVolts(phiVel, phiAcc, theta, phi, false);

    thetaVolts += (wantedTheta - theta) * skP_ + (thetaVel - shoulderVel) * skV_;
    phiVolts += (wantedPhi - phi) * ekP_ + (phiVel - elbowVel) * ekV_;

    cout << "tv: " << thetaVolts << ", pv: " << phiVolts << endl;

}

void TwoJointArm::followJointSpaceProfile(double time)
{
    tuple<double, double, double> shoulderProfile = shoulderTraj_.getProfile(time);
    tuple<double, double, double> elbowProfile = elbowTraj_.getProfile(time);

    double wantedTheta = get<0>(shoulderProfile);
    double wantedPhi = get<0>(elbowProfile);
    double thetaVel = get<1>(shoulderProfile);
    double phiVel = get<1>(elbowProfile);
    double thetaAcc = get<2>(shoulderProfile);
    double phiAcc = get<2>(elbowProfile);

    double theta = wantedTheta; // getTheta();
    double phi = wantedPhi; // getPhi();
    double shoulderVel = thetaVel; // shoulderMaster_.GetSelectedSensorVelocity()* (10.0 / 2048.0) * 360.0 * TwoJointArmConstants::MOTOR_TO_SHOULDER_RATIO;
    double elbowVel = phiVel; // elbowMaster_.GetSelectedSensorVelocity()* (10.0 / 2048.0) * 360.0 * TwoJointArmConstants::MOTOR_TO_ELBOW_RATIO - shoulderVel * TwoJointArmConstants::SHOULDER_TO_ELBOW_RATIO;

    if (thetaVel == 0 && phiVel == 0 && thetaAcc == 0 && phiAcc == 0)
    {
        position_ = setPosition_;
        if (abs(theta - wantedTheta) > 20 || abs(phi - wantedPhi) > 20) //TODO get values
        {
            state_ = STOPPED;
        }
        else if (abs(theta - wantedTheta) > 10 || abs(phi - wantedPhi) > 10)
        {
            shoulderTraj_.generateTrajectory(theta, wantedTheta, thetaVel);
            elbowTraj_.generateTrajectory(phi, wantedPhi, phiVel);
            jsst_ = time_;
            state_ = FOLLOWING_JOINT_SPACE_PROFILE;
        }
        else
        {
            state_ = HOLDING_POS;
        }
    }

    double thetaVolts = calcShoulderVolts(thetaVel, thetaAcc, theta, phi, false); //TODO check if has cone is viable or necesary
    double phiVolts = calcElbowVolts(phiVel, phiAcc, theta, phi, false);

    thetaVolts += (wantedTheta - theta) * skP_ + (thetaVel - shoulderVel) * skV_;
    phiVolts += (wantedPhi - phi) * ekP_ + (phiVel - elbowVel) * ekV_;

}

void TwoJointArm::home(double time)
{

    double shoulderVel = 0; // shoulderMaster_.GetSelectedSensorVelocity()* (10.0 / 2048.0) * 360.0 * TwoJointArmConstants::MOTOR_TO_SHOULDER_RATIO;
    double elbowVel = 0; // elbowMaster_.GetSelectedSensorVelocity()* (10.0 / 2048.0) * 360.0 * TwoJointArmConstants::MOTOR_TO_ELBOW_RATIO - shoulderVel * TwoJointArmConstants::SHOULDER_TO_ELBOW_RATIO;
    double theta = getTheta();
    double phi = getPhi();

    if (!homing_.first)
    {

        shoulderTraj_.generateTrajectory(theta, 0, shoulderVel);
        jsst_ = time_;
        time = 0;
        homing_.first = true;
    }

    if (!homing_.second && TwoJointArmConstants::UPPER_ARM_LENGTH * cos(theta * 3.1415 / 180.0) > (TwoJointArmConstants::FOREARM_LENGTH + TwoJointArmConstants::EE_LENGTH + 0.1))
    {
        elbowTraj_.generateTrajectory(phi, 180, elbowVel);
        homing_.second = true;
    }

    tuple<double, double, double> shoulderProfile = shoulderTraj_.getProfile(time);
    tuple<double, double, double> elbowProfile = elbowTraj_.getProfile(time);

    double wantedTheta = get<0>(shoulderProfile);
    double thetaVel = get<1>(shoulderProfile);
    double thetaAcc = get<2>(shoulderProfile);

    double thetaVolts = calcShoulderVolts(thetaVel, thetaAcc, theta, phi, false); //TODO check if has cone is viable or necesary
    thetaVolts += (wantedTheta - theta) * skP_ + (thetaVel - shoulderVel) * skV_;

    if (homing_.second)
    {
        double wantedPhi = get<0>(elbowProfile);
        double phiVel = get<1>(elbowProfile);
        double phiAcc = get<2>(elbowProfile);

        double phiVolts = calcElbowVolts(phiVel, phiAcc, theta, phi, false);
        phiVolts += (wantedPhi - phi) * ekP_ + (phiVel - elbowVel) * ekV_;


        if (thetaVel == 0 && phiVel == 0 && thetaAcc == 0 && phiAcc == 0)
        {
            homing_ = { false, false };
            posUnknown_ = false;
            position_ = TwoJointArmProfiles::STOWED;
            if (abs(theta - wantedTheta) > 20 || abs(phi - wantedPhi) > 20) //TODO get values
            {
                state_ = STOPPED;
            }
            else if (abs(theta - wantedTheta) > 10 || abs(phi - wantedPhi) > 10)
            {
                shoulderTraj_.generateTrajectory(theta, wantedTheta, thetaVel);
                elbowTraj_.generateTrajectory(phi, wantedPhi, phiVel);
                jsst_ = time_;
                state_ = FOLLOWING_JOINT_SPACE_PROFILE;
            }
            else
            {
                state_ = HOLDING_POS;
            }
        }
    }
}

double TwoJointArm::calcShoulderVolts(double vel, double acc, double theta, double phi, bool hasCone)
{
    double gravityTorque = calcShoulderGravityTorque(theta, phi, hasCone);
    double rotInertia = calcShoulderRotInertia(phi, hasCone);
    double accTorque = acc * rotInertia;

    double torque = gravityTorque + accTorque;
    torque *= TwoJointArmConstants::MOTOR_TO_SHOULDER_RATIO * 0.5;
    double torqueVolts = (torque - 0.948) / 0.3055;
    //double torqueVolts = torque * (calcR(prevShoulderVolts_) / calcKT(prevShoulderVolts_));
    //double torqueVolts = torque * (0.04669261 / 0.01824903);
    //TODO see which is best
    //COULDO torque with non-linear relationship

    double velVolts = (vel)*TwoJointArmConstants::SHOULDER_KV + TwoJointArmConstants::SHOULDER_KVI; //If gotten directly, all good. If using motor curves remember to convert to radians/sec and use gear ratio

    prevShoulderVolts_ = torqueVolts + velVolts;
    return torqueVolts + velVolts;
}
double TwoJointArm::calcElbowVolts(double vel, double acc, double theta, double phi, double hasCone)
{
    double gravityTorque = calcElbowGravityTorque(theta, phi, hasCone);
    double rotInertia = calcElbowRotInertia(hasCone);
    double accTorque = acc * rotInertia;

    double torque = gravityTorque + accTorque;
    torque *= TwoJointArmConstants::MOTOR_TO_ELBOW_RATIO * 0.5;
    double torqueVolts = (torque - 0.948) / 0.3055;
    //double torqueVolts = torque * (calcR(prevElbowVolts_) / calcKT(prevElbowVolts_));
    //double torqueVolts = torque * (0.04669261 / 0.01824903);
    //TODO see which is best
    //COULDO torque with non-linear relationship

    double velVolts = (vel)*TwoJointArmConstants::ELBOW_KV + TwoJointArmConstants::ELBOW_KVI; //If gotten directly, all good. If using motor curves remember to convert to radians/sec and use gear ratio

    prevElbowVolts_ = torqueVolts + velVolts;
    return torqueVolts + velVolts;
}

double TwoJointArm::calcShoulderRotInertia(double phi, bool hasCone)
{
    double shoulderToForearmCom = sqrt(TwoJointArmConstants::UPPER_ARM_LENGTH * TwoJointArmConstants::UPPER_ARM_LENGTH + TwoJointArmConstants::FOREARM_COM_DIST * TwoJointArmConstants::FOREARM_COM_DIST
        - 2 * TwoJointArmConstants::UPPER_ARM_LENGTH * TwoJointArmConstants::FOREARM_COM_DIST * cos((180 - phi) * (3.1415 / 180.0)));
    double noConeI = TwoJointArmConstants::SHOULDER_I + (TwoJointArmConstants::FOREARM_I * shoulderToForearmCom * shoulderToForearmCom);

    if (hasCone)
    {
        double shoulderToConeDist = sqrt(TwoJointArmConstants::UPPER_ARM_LENGTH * TwoJointArmConstants::UPPER_ARM_LENGTH + TwoJointArmConstants::FOREARM_LENGTH * TwoJointArmConstants::FOREARM_LENGTH
            - 2 * TwoJointArmConstants::UPPER_ARM_LENGTH * TwoJointArmConstants::FOREARM_LENGTH * cos((180 - phi) * (3.1415 / 180.0)));
        return noConeI + GeneralConstants::CONE_M * shoulderToConeDist * shoulderToConeDist;
    }
    else
    {
        return noConeI;
    }
}
double TwoJointArm::calcElbowRotInertia(bool hasCone)
{
    if (hasCone)
    {
        return TwoJointArmConstants::ELBOW_I + (TwoJointArmConstants::FOREARM_LENGTH * TwoJointArmConstants::FOREARM_LENGTH * GeneralConstants::CONE_M);
    }
    else
    {
        return TwoJointArmConstants::ELBOW_I;
    }

}

double TwoJointArm::calcShoulderGravityTorque(double theta, double phi, bool hasCone)
{
    double upperArmTorque = TwoJointArmConstants::UPPER_ARM_COM_DIST * TwoJointArmConstants::UPPER_ARM_M * GeneralConstants::g * sin(theta * 3.1415 * 180.0);

    //COULDO move these calcs into ArmKinematics
    double upperArmX = -TwoJointArmConstants::UPPER_ARM_LENGTH * sin((-theta) * (3.1415 / 180.0));
    double upperArmY = TwoJointArmConstants::UPPER_ARM_LENGTH * cos((-theta) * (3.1415 / 180.0));

    double secondJointAng_baseCords = theta + phi;

    double forearmCOMX = upperArmX - TwoJointArmConstants::FOREARM_COM_DIST * sin((-secondJointAng_baseCords) * (3.1415 / 180.0));
    double forearmCOMY = upperArmY + TwoJointArmConstants::FOREARM_COM_DIST * cos((-secondJointAng_baseCords) * (3.1415 / 180.0));

    double forearmTorque = sqrt(forearmCOMX * forearmCOMX * forearmCOMY * forearmCOMY) * TwoJointArmConstants::FOREARM_M * GeneralConstants::g * sin((3.1415 / 2) - atan2(forearmCOMY, forearmCOMX));

    if (hasCone)
    {
        pair<double, double> xy = armKinematics_.angToXY(theta, phi);
        double coneTorque = sqrt(xy.first * xy.first + xy.second * xy.second) * GeneralConstants::CONE_M * GeneralConstants::g * sin((3.1415 / 2) - atan2(xy.second, xy.first));
        return upperArmTorque + forearmTorque + coneTorque;
    }
    else
    {
        return upperArmTorque + forearmTorque;
    }
}
double TwoJointArm::calcElbowGravityTorque(double theta, double phi, bool hasCone)
{
    double forearmTorque = TwoJointArmConstants::FOREARM_COM_DIST * TwoJointArmConstants::FOREARM_M * GeneralConstants::g * sin((theta + phi) * (3.1415 / 180.0));

    if (hasCone)
    {
        return forearmTorque + TwoJointArmConstants::FOREARM_LENGTH * GeneralConstants::CONE_M * GeneralConstants::g * sin((theta + phi) * (3.1415 / 180.0));
    }
    else
    {
        return forearmTorque;
    }
}

double TwoJointArm::calcKT(double volts)
{
    return 0.000646581 * volts * volts - 0.0147623 * volts + 0.102916;
}

double TwoJointArm::calcR(double volts)
{
    return 0.000758486 * volts * volts - 0.0187702 * volts + 0.163911;
}

double TwoJointArm::getTheta()
{
    return 0; //TODO with encoders, don't forget switching sides
}

double TwoJointArm::getPhi()
{
    return 0; //TODO with encoders, don't forget switching sides
}