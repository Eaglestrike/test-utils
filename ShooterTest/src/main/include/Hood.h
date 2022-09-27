#pragma once
#include <ctre/Phoenix.h>
#include "TrajectoryCalc.h"
#include <string>

class Hood{
    public:
        void goToPose(double ticks);
        void followPath(std::string path);
        double getPose();

    private:
        void initTrajectory();
        void move(double setPos_);

        //i cri
        bool initTrajectory_ = false;
        double setTrajectoryPos_ = 0;

        double kP = 0.0001;
        double kP_ = 0.0008; //wtf
        double kD = 0;
        double kV = 1 / ShooterConstants::HOOD_NEG_FF;
        double kA = 0;
        double kVI = ShooterConstants::HOOD_NEG_FF_INTERCEPT;

        WPI_TalonFX hoodMotor_{19};
        TrajectoryCalc trajectoryCalc_{100000, 500000 * 1.5, kP, kD, kV, kA, kVI};
};
