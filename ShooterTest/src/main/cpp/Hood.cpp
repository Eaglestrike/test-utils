#include "Hood.h"

void Hood::goToPose(double ticks) {
   // move(ticks);
   units::radian_t degrees = ticksToAngle(ticks);
   goToPoseSysid(degrees);
}

double Hood::getPose() {
    return hoodMotor_.GetSelectedSensorPosition();
}

double Hood::getPIDError() {
    return profiledController.GetPositionError().value();
}

void Hood::zero() {
    hoodMotor_.SetSelectedSensorPosition(0);
}

void Hood::setVoltage(double voltage) {
    hoodMotor_.SetVoltage(units::volt_t{voltage});
}

units::radian_t Hood::ticksToAngle(double ticks) {
    return units::radian_t{ticks / ShooterConstants::TICKS_PER_HOOD_DEGREE * wpi::numbers::pi / 180};
}


void Hood::goToPoseSysid(units::radian_t angle) {

    auto desiredAcceleration = (profiledController.GetSetpoint().velocity - lastSpeed) / (frc::Timer::GetFPGATimestamp() - lastTime);
    units::volt_t volts = units::volt_t{profiledController.Calculate(ticksToAngle(hoodMotor_.GetSelectedSensorPosition()), angle)} +
        feedforward.Calculate(profiledController.GetSetpoint().velocity, desiredAcceleration);

    frc::SmartDashboard::PutNumber("profiledPID calculator", profiledController.Calculate(ticksToAngle(hoodMotor_.GetSelectedSensorPosition()), angle));
    frc::SmartDashboard::PutNumber("feedforward calculator:", 
        feedforward.Calculate(profiledController.GetSetpoint().velocity, desiredAcceleration).value());


    //TODO check encoder/voltage directionality of hood
    if(hoodMotor_.GetSelectedSensorPosition() < ShooterConstants::MAX_HOOD_TICKS && volts.value() < 0) {
        hoodMotor_.SetVoltage(units::volt_t(0));
        frc::SmartDashboard::PutNumber("Hood Volts", 0);
    }
    else if(hoodMotor_.GetSelectedSensorPosition() > 0 && volts.value() > 0) {
        hoodMotor_.SetVoltage(units::volt_t(0));
        frc::SmartDashboard::PutNumber("Hood Volts", 0);
    }
    else {
        hoodMotor_.SetVoltage(volts);
        frc::SmartDashboard::PutNumber("Hood Volts", volts.value());
    }

    lastSpeed = profiledController.GetSetpoint().velocity;
    lastTime = frc::Timer::GetFPGATimestamp();

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