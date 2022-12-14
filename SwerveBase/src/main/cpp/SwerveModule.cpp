#include "SwerveModule.h"

/**
 * Iniitialize a swerve module. Mostly setting ports, also if the encoder has an offset/motor is inverted
**/
SwerveModule::SwerveModule(int angMotorPort, int speedMotorPort, int canCoderPort, double offset, bool inverted) : angleMotor_{angMotorPort, "rio"},
speedMotor_{speedMotorPort, "rio"}, canCoder_{canCoderPort, "rio"}, offset_{offset}
{
    speedMotor_.SetInverted(inverted);

    angleMotor_.SetSelectedSensorPosition(0);
    speedMotor_.SetSelectedSensorPosition(0);

    angleMotor_.SetNeutralMode(NeutralMode::Brake);
    speedMotor_.SetNeutralMode(NeutralMode::Brake);
}

/**
 * Converts the raw encoder speed of the driving swerve motor to its speed in meters per second
 * @returns velocity in mps
**/
units::meters_per_second_t SwerveModule::talonVelToMps(double vel)
{
    double wheel_radius = 0.05;                      // in meters
    double meters_per_rev = wheel_radius * 2 * M_PI; // wheel circumberence
    double ticks_per_rev = 12650;
    return units::meters_per_second_t{vel / 0.1 * (meters_per_rev / ticks_per_rev)};
}

/**
 * Gets the drive speed and angle of a swerve module
 * @returns a SwerveModuleState encapsulating that info
**/
// TODO: check input modulus of Rotation2d
frc::SwerveModuleState SwerveModule::getState()
{
    frc::SwerveModuleState state; // TODO: can this be made inline?
    state.speed = talonVelToMps(speedMotor_.GetSelectedSensorVelocity());
    state.angle = frc::Rotation2d{units::angle::degree_t{getYaw()}};
    return state;
}

/**
 * @returns the "optimized" version of the @param state 
**/
frc::SwerveModuleState SwerveModule::getOptState(frc::SwerveModuleState state)
{
    double yaw = frc::InputModulus(canCoder_.GetAbsolutePosition() + offset_, -180.0, 180.0);
    frc::SwerveModuleState opt_state = frc::SwerveModuleState::Optimize(state, units::degree_t(yaw));
    return opt_state;
}


void SwerveModule::setAngMotorVoltage(double volts)
{
    angleMotor_.SetVoltage(units::volt_t{volts});
}

void SwerveModule::setSpeedMotor(double power)
{
    speedMotor_.Set(ControlMode::PercentOutput, power);
}

void SwerveModule::setSpeedMotorVolts(double volts) {
    speedMotor_.SetVoltage(units::volt_t{volts});
}

//note: in raw ticks, not mps
double SwerveModule::getVelocity()
{
    return speedMotor_.GetSelectedSensorVelocity();
}

//use this for getting velocity in mps
units::meters_per_second_t SwerveModule::getVelocityMPS() {
    return talonVelToMps(getVelocity());
}

//in degrees
double SwerveModule::getYaw()
{
    // std::cout << "abs pos: " << canCoder_.GetAbsolutePosition();
    // std::cout << "offset: " << offset_;
    return canCoder_.GetAbsolutePosition() + offset_;
}