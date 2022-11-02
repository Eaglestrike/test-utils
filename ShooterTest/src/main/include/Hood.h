#pragma once
#include <ctre/Phoenix.h>
#include "TrajectoryCalc.h"
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/controller/ProfiledPIDController.h>
#include <string>
#include <frc/Timer.h>

class Hood{
    public:
        void goToPose(double ticks);
        double getPose();
        double getPIDError();
        void zero();
        void setVoltage(double voltage);

        void followPath(std::string path); //TODO: write

    private:
        //Alex pid stuff
        void move(double setPos_); 

        bool initTrajectory_ = false;
        double setTrajectoryPos_ = 0;

        double kP = 0.0001;
        double kP_ = 0.0008; 
        double kD = 0;
        double kV = 1 / ShooterConstants::HOOD_NEG_FF;
        double kA = 0;
        double kVI = ShooterConstants::HOOD_NEG_FF_INTERCEPT;  

        TrajectoryCalc trajectoryCalc_{100000, 500000 * 1.5, kP, kD, kV, kA, kVI};


        //wpilib pid stuff
        //constants in SysidConstants 
        void goToPoseSysid(units::radian_t angle); 

        units::radian_t ticksToAngle(double ticks);

        units::radians_per_second_t lastSpeed = 0_rad_per_s;
        units::second_t lastTime = frc::Timer::GetFPGATimestamp();

        frc::PIDController feedback{SysidConstants::kp, SysidConstants::ki, SysidConstants::kd};  
        frc::SimpleMotorFeedforward<units::radians> feedforward{SysidConstants::ks, SysidConstants::kv, SysidConstants::ka};
        frc::ProfiledPIDController<units::radians> profiledController{SysidConstants::kp, SysidConstants::ki, SysidConstants::kd, 
            frc::TrapezoidProfile<units::radians>::Constraints{SysidConstants::maxVel, SysidConstants::maxAcc}};

        WPI_TalonFX hoodMotor_{19};
        
};
