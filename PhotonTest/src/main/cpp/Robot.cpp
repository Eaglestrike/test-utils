
#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {}

void Robot::RobotPeriodic() {
  // nt::NetworkTableInstance::GetDefault().Flush(); //guy on chiefdelphi says this helps with updates?

  frc::SmartDashboard::PutNumber("Latecy", vision_.getLatency());
  frc::SmartDashboard::PutBoolean("Has Targerts", vision_.latestResult().HasTargets());
  frc::SmartDashboard::PutNumber("Pose x", vision_.getPose().X().value());
  frc::SmartDashboard::PutNumber("Pose y", vision_.getPose().Y().value());
  frc::SmartDashboard::PutNumber("Pose z", vision_.getPose().Z().value());

}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {}

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
