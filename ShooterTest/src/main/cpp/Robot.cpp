#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>


void Robot::RobotInit() {}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  hood_.zero();
}

void Robot::TeleopPeriodic() {
  hoodPose = frc::SmartDashboard::GetNumber("Set hood ticks", hoodPose);
  frc::SmartDashboard::PutNumber("Set hood ticks", hoodPose);

  hoodVoltage = frc::SmartDashboard::GetNumber("Set hood volts", hoodVoltage);
  frc::SmartDashboard::PutNumber("Set hood volts", hoodVoltage);

  frc::SmartDashboard::PutNumber("Hood pose", hood_.getPose());
  frc::SmartDashboard::PutNumber("Pid error", hood_.getPIDError());

  //A
  if (xbox_.GetRawButton(1)) {
    hood_.goToPose(hoodPose);
  }
  //B
  else if (xbox_.GetRawButton(2)) {
    hood_.setVoltage(hoodVoltage);
  }

 // todo: add file & command for hood path follow
  
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
