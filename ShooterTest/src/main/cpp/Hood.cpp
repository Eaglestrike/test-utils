#include "Hood.h"

void Hood::goToPose(double ticks) {
    move(ticks);
}

double Hood::getPose() {
    return hoodMotor_.GetSelectedSensorPosition();
}

void Hood::move(double setPos_) {
    //double volts = calcPID();

    //setPos_ = frc::SmartDashboard::GetNumber("InA", 0);

    double volts;
    if(abs(setPos_ - setTrajectoryPos_) > 50 && initTrajectory_) //TODO get value
    {
        setTrajectoryPos_ = setPos_;
        //double pos = get<2>(trajectoryCalc_.getProfile());
        //double vel = get<1>(trajectoryCalc_.getProfile());
        double pos = hoodMotor_.GetSelectedSensorPosition();
        double vel = hoodMotor_.GetSelectedSensorVelocity() * 10;

        trajectoryCalc_.generateTrajectory(pos, setPos_, vel);
    }
    else if(!initTrajectory_)
    {
        initTrajectory_ = true;
        setTrajectoryPos_ = setPos_;

        double pos = hoodMotor_.GetSelectedSensorPosition();
        double vel = hoodMotor_.GetSelectedSensorVelocity() * 10;

        trajectoryCalc_.generateTrajectory(pos, setPos_, vel);
    }

    if(initTrajectory_)
    {
        double pos = hoodMotor_.GetSelectedSensorPosition();
        double vel = hoodMotor_.GetSelectedSensorVelocity() * 10;
        //volts = trajectoryCalc_.calcPower(pos, vel)// + ShooterConstants::HOOD_WEIGHT_FF;

        tuple<double, double, double> profile = trajectoryCalc_.getProfile();

        double profileVel = get<1>(profile);
        double profileAcc = get<0>(profile);
        //double profilePos = get<2>(profile);
        double kVVolts;
        if(profileVel < 0)
        {
            kVVolts = (profileVel - ShooterConstants::HOOD_NEG_FF_INTERCEPT) / ShooterConstants::HOOD_NEG_FF;
        }
        else if(profileVel > 0)
        {
            kVVolts = (profileVel - ShooterConstants::HOOD_POS_FF_INTERCEPT) / ShooterConstants::HOOD_POS_FF;
        }
        else
        {
            //kVVolts = 0;
            kVVolts = ShooterConstants::HOOD_WEIGHT_FF;
        }

        if(profileVel == 0 && profileAcc == 0 && vel < 50) //HERE
        {
            volts = ((setPos_ - pos) * kP_) + ShooterConstants::HOOD_WEIGHT_FF;
            //volts = ((profilePos - pos) * kP_) + ShooterConstants::HOOD_WEIGHT_FF;
        }
        else
        {
            volts = ((get<2>(profile) - pos) * kP) + ((profileVel - vel) * kD) + kVVolts + (profileAcc * kA);
        }
        

    }
    else
    {
        volts = 0;
    }


    if(hoodMotor_.GetSelectedSensorPosition() < ShooterConstants::MAX_HOOD_TICKS && volts < 0)
    {
        hoodMotor_.SetVoltage(units::volt_t(0));
    }
    else if(hoodMotor_.GetSelectedSensorPosition() > 0 && volts > 0)
    {
        hoodMotor_.SetVoltage(units::volt_t(0));
    }
    else
    {
        hoodMotor_.SetVoltage(units::volt_t(volts));
    }
}