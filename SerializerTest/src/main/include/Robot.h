// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/XboxController.h>

#include "Constants.h"
#include "Serializer.hpp"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;
 
 private:
  Serializer serializer{Constants::SERIALIZER_MOTOR_ID,
                 Constants::SERIALIZER_ENCODER_INPUT_PIN,
                 Constants::SERIALIZER_LIMIT_SWITCH_PIN,
                 Constants::SERIALIZER_DEFAULT_VOLTAGE,
                 Constants::SERIALIZER_MAXIMUM_VOLTAGE,
                 Constants::SERIALIZER_CONE_ANGLE_THRESH,
                 Constants::SERIALIZER_ANG_TO_TARGET,
                 Constants::SERIALIZER_KP,
                 Constants::SERIALIZER_KI,
                 Constants::SERIALIZER_KD,
                 Constants::SERIALIZER_ERR_TOL,
                 Constants::SERIALIZER_ERR_DER_TOL};

  // this controller is used for testing
  frc::XboxController sampleController{0};
};
