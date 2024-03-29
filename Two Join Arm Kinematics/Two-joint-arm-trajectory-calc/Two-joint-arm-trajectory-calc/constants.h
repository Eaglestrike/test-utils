#pragma once

#include <math.h>
#include "string"

using namespace std;

#define pi 3.1415

namespace GeneralConstants
{
    const double g = 9.81;

    const int MAX_RPM = 6380;
    const int TICKS_PER_ROTATION = 2048;

    const double FREE_SPEED = 6380;
    const double FREE_CURRENT = 1.5;
    const double STALL_CURRENT = 257;
    const double MAX_VOLTAGE = 12;

    const double RESISTANCE = MAX_VOLTAGE / STALL_CURRENT;
    const double Kv = ((FREE_SPEED * 2 * pi) / 60) / (MAX_VOLTAGE - FREE_CURRENT * RESISTANCE);

    const double FIELD_WIDTH = 8.2296;
    const double FIELD_LENGTH = 16.4592;

    const double CONE_M = 0.652;

}

namespace InputConstants
{
    const int LJOY_PORT = 0;
    const int LJOY_X = 0;
    const int LJOY_Y = 1;

    const int RJOY_PORT = 1;
    const int RJOY_X = 0;
    const int RJOY_Y = 1;

    const int XBOX_PORT = 2;
    const int XBOX_LJOY_X = 0;
    const int XBOX_LJOY_Y = 1;
    const int XBOX_LTRIGGER = 4; //TODO check values
    const int XBOX_RTRIGGER = 5;
    const int XBOX_RJOY_X = 2;
    const int XBOX_RJOY_Y = 3;

    const int OUTAKE_BUTTON = 3;

    const int A_BUTTON = 1;
    const int B_BUTTON = 2;
    const int X_BUTTON = 3;
    const int Y_BUTTON = 4;
    const int L_BUMPER = 5;
    const int R_BUMPER = 6;
    const int CLIMB_MODE_TOGGLE_BUTTON = 7;
    const int FIELD_ORIENT_BUTTON = 8;

}

namespace SwerveConstants
{
    const double WIDTH = 29; //Change to 22 
    const double LENGTH = 29;
    const double WHEEL_DIAGONAL = 0.8128;
    const double TREAD_RADIUS = 0.0508;
    const double DRIVE_GEAR_RATIO = 1 / 6.12;

    //const double trPosAngle = atan2((SwerveConstants::WIDTH/2), (SwerveConstants::LENGTH/2));
    //const double tlPosAngle = -trPosAngle;
    //const double brPosAngle = 180 - trPosAngle;
    //const double blPosAngle = trPosAngle - 180;

    const int TR_DRIVE_ID = 13; //13, 3
    const int TL_DRIVE_ID = 11; //11, 1
    const int BR_DRIVE_ID = 18; //18, 4
    const int BL_DRIVE_ID = 22; //15, 22

    const int TR_TURN_ID = 14; //14, 5
    const int TL_TURN_ID = 12; //12, 7
    const int BR_TURN_ID = 17; //17, 10
    const int BL_TURN_ID = 19; //16, 19

    const int TR_CANCODER_ID = 62; //62, 2
    const int TL_CANCODER_ID = 10; //10, 9
    const int BR_CANCODER_ID = 8; //8, 8
    const int BL_CANCODER_ID = 6; //42, 6

    const double TR_CANCODER_OFFSET = 19.77; //19.77, 
    const double TL_CANCODER_OFFSET = 109.952; //109.952
    const double BR_CANCODER_OFFSET = 197.5; //197.5
    const double BL_CANCODER_OFFSET = -139.92 + 180; // 356.39, -184 + 90, -236.3 + 180, idk before this it was -236

    const double MAX_LA = 3;
    const double MAX_LV = 4;
    const double MAX_AA = 270;
    const double MAX_AV = 450;

    const double klV = 0.489016;
    const double klVI = -0.32683;
    const double klA = 0;
    const double klP = 0.1; //0.05
    const double klD = 0;

    const double kaV = 34.2064;
    const double kaVI = -25.4095;
    const double kaA = 0;
    const double kaP = 0.02; //0.008
    const double kaD = 0;

}

namespace TwoJointArmConstants
{
    const double UPPER_ARM_LENGTH = 0.762; //TODO get values 0.635
    const double FOREARM_LENGTH = 0.762; //0.8636
    const double EE_LENGTH = 0.2794; //0.2794
    const double MOUNTING_HEIGHT = 0.508; //0.508

    const double SHOULDER_MIN_ANG = -120;
    const double SHOULDER_MAX_ANG = 120;
    const double ELBOW_MIN_ANG = 0;
    const double ELBOW_MAX_ANG = 200;

    const double SHOULDER_ARM_MAX_VEL = 720;
    const double ELBOW_ARM_MAX_VEL = 720;
    const double SHOULDER_ARM_MAX_ACC = 360;
    const double ELBOW_ARM_MAX_ACC = 360;

    const int SHOULDER_MASTER_ID = 20; //TODO get ids
    const int SHOULDER_SLAVE_ID = 1;
    const int ELBOW_MASTER_ID = 14;
    const int ELBOW_SLAVE_ID = 3;
    const int SHOULDER_BRAKE_ID = 4;
    const int ELBOW_BRAKE_ID = 5;

    const double UPPER_ARM_I = 0.19;
    const double FOREARM_I = 0.15;

    const double UPPER_ARM_M = 0.114; //2.53
    const double FOREARM_M = 0.114; //1.41

    const double UPPER_ARM_COM_DIST = 0.381; //0.304
    const double FOREARM_COM_DIST = 0.381; //0.26

    const double SHOULDER_I = UPPER_ARM_I + UPPER_ARM_M * UPPER_ARM_COM_DIST * UPPER_ARM_COM_DIST;
    const double ELBOW_I = FOREARM_I + FOREARM_M * FOREARM_COM_DIST * FOREARM_COM_DIST;

    const double SHOULDER_KV = 0;
    const double SHOULDER_KVI = 0;
    const double ELBOW_KV = 0;
    const double ELBOW_KVI = 0;

    const double SHOULDER_TO_ELBOW_RATIO = 1.0 / 1.0; //30:54
    const double MOTOR_TO_SHOULDER_RATIO = 1.0 / 100.0; // 1:194.4, 1:100
    const double MOTOR_TO_ELBOW_RATIO = 1.0 / 50.0; //1:40.5 for motor to shoulder area, 1:72.9 for motor to elbow joint

    //Stowed
    const double sX = 0.355439218;
    const double sY = -0.184396634;
    const double sTheta = 6.42;
    const double sPhi = 154.35;

    //Ground intake
    const double giX = 0.40564;
    const double giY = -0.53042;
    const double giTheta = 59.59;
    const double giPhi = 129.87;

    //player station
    const double psX = 0.54164;
    const double psY = 0.43525;
    const double psTheta = -29.64;
    const double psPhi = 127.4;

    //mid
    const double mX = 1.02512;
    const double mY = 0.29191;
    const double mTheta = 20;
    const double mPhi = 90;

    //high
    const double hX = 1.38078;
    const double hY = 0.54399;
    const double hTheta = 59.18;
    const double hPhi = 16.16;

    //intake
    const double iX = 0.33037399;
    const double iY = -0.537556202;
    const double iTheta = 62.4;
    const double iPhi = 133.21;

    const double ANGLE_ERROR_THRESHOLD = 5; //TODO check value

}