#include "Serializer.hpp"

Serializer::Serializer(const int motorID,
                       const int encoderPinNo,
                       const int limitSwitchPinNo,
                       const int defaultVolts,
                       const double maxVolts,
                       const double angleTresh,
                       const double angleToTarget,
                       const double kp,
                       const double ki,
                       const double kd,
                       const double errorTolerance,
                       const double errorDerivativeTolerance)
  : m_srx(motorID), m_encoder(encoderPinNo), m_limitSwitchInput(limitSwitchPinNo),
    m_maxVolts(maxVolts), m_curVolts(defaultVolts), m_angleThresh(angleTresh),
    m_angleToTarget(angleToTarget), m_kp(kp), m_ki(ki), m_kd(kd) {
  // set PID setpoint
  m_controller.SetTolerance(errorTolerance, errorDerivativeTolerance);
}

void Serializer::RobotInit() {
  // display max, cur voltage, angles, PID
  frc::SmartDashboard::PutNumber("Serializer Maximum Voltage", m_maxVolts);
  frc::SmartDashboard::PutNumber("Serializer Current Voltage", m_curVolts);
  frc::SmartDashboard::PutNumber("Serializer Angle Thresh", m_angleThresh);
  frc::SmartDashboard::PutNumber("Serializer Target Ang", m_angleToTarget);
  frc::SmartDashboard::PutNumber("Serializer P", m_kp);
  frc::SmartDashboard::PutNumber("Serializer I", m_ki);
  frc::SmartDashboard::PutNumber("Serializer D", m_kd);
  frc::SmartDashboard::PutBoolean("Serializer Ready To Grab", IsPositioned());

  // reset encoder value, PID, state
  Reset();
}

void Serializer::AutonomousPeriodic() {
  TeleopPeriodic();
}

void Serializer::TeleopPeriodic() {
  // get encoder value
  double encoderValue = m_encoder.Get().to<double>();
  frc::SmartDashboard::PutBoolean("Serializer Encoder Connected", m_encoder.IsConnected());
  frc::SmartDashboard::PutNumber("Serializer Encoder Value", encoderValue);
  frc::SmartDashboard::PutBoolean("Serializer Ready To Grab", IsPositioned());

  // check if cone pressed limit switch and update switch if it did
  bool limitSwitchPressed = m_limitSwitchInput.Get();
  
  // state machine
  switch (m_curState) {
    case FINDING_CONE:
      if (limitSwitchPressed) {
        // update state after being switched once
        m_curState = CONE_SWITCHED;
        m_encoderValueWhenHit = encoderValue;

        // when switched once, target value assumes that it is the inside of the cone
        // that switches the switch. This will change if the switch gets pressed again (i.e. goes
        // into CONE_FOUND_TWICE)
        m_targetValue = encoderValue + m_angleToTarget;
      }
      m_srx.SetVoltage(units::volt_t{m_curVolts});
      break;
    case CONE_SWITCHED:
      if (limitSwitchPressed) {
        // make sure that it is actually pressing the limit switch twice
        if (std::abs(encoderValue - m_encoderValueWhenHit) > m_angleThresh*(2/3.0)){
          m_curState = CONE_REPOSITIONING;

          // when switched twice, this means that the inside of the cone definitely switched the switch
          // so update the target value
          m_targetValue = encoderValue + m_angleToTarget;
          m_controller.SetSetpoint(m_targetValue);
        }
      }
      if (std::abs(encoderValue - m_encoderValueWhenHit) > m_angleThresh) {
        m_curState = CONE_REPOSITIONING; 
        m_controller.SetSetpoint(m_targetValue);
      }
      break;
    case CONE_REPOSITIONING:
    {
      double pidOutput = m_controller.Calculate(encoderValue, m_targetValue);
      pidOutput = std::clamp(pidOutput, -m_maxVolts, m_maxVolts);

      if (m_controller.AtSetpoint()) {
        m_curState = CONE_POSITIONED;
      }
      m_srx.SetVoltage(units::volt_t{pidOutput});
      break;
    }
    default: // off or positioned
      m_srx.SetVoltage(units::volt_t{0});
      break;
  }
}

void Serializer::DisabledInit() {
  // stop motor
  m_srx.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
  m_srx.SetVoltage(units::volt_t{0});

  // reset states, PID, encoder value
  Reset();
}

void Serializer::DisabledPeriodic() {
  // set voltage, angles, PID
  m_maxVolts = frc::SmartDashboard::GetNumber("Serializer Maximum Voltage", m_maxVolts);
  m_curVolts = frc::SmartDashboard::GetNumber("Serializer Current Voltage", m_curVolts);
  m_angleThresh = frc::SmartDashboard::GetNumber("Serializer Angle Thresh", m_angleThresh);
  m_angleToTarget = frc::SmartDashboard::GetNumber("Serializer Target Ang", m_angleToTarget);
  m_kp = frc::SmartDashboard::GetNumber("Serializer P", m_kp);
  m_ki = frc::SmartDashboard::GetNumber("Serializer I", m_ki);
  m_kd = frc::SmartDashboard::GetNumber("Serializer D", m_kd);
  
  // clamp volts
  m_curVolts = std::clamp(m_curVolts, -m_maxVolts, m_maxVolts);

  // get encoder value
  double encoderValue = m_encoder.Get().to<double>();
  frc::SmartDashboard::PutBoolean("Serializer Encoder Connected", m_encoder.IsConnected());
  frc::SmartDashboard::PutNumber("Serializer Encoder Value", encoderValue);
  frc::SmartDashboard::PutBoolean("Serializer Ready To Grab", IsPositioned());

  // set PID values
  m_controller.SetPID(m_kp, m_ki, m_kd);
}

/**
 * Turns on turntable wheel; makes it start spinning so that
 * it's ready to detect the cone.
 * 
 * (call this after cone is intaken)
*/
void Serializer::TurnOn() {
  m_curState = FINDING_CONE;
}

/**
 * Turns off the turntable wheel
*/
void Serializer::TurnOff() {
  m_curState = OFF;
}

/**
 * If the arm is ready to pick up the cone
*/
bool Serializer::IsPositioned() {
  return m_curState == CONE_POSITIONED;
}

/**
 * Resets the encoder value
*/
void Serializer::ResetEncoderValue() {
  m_encoder.Reset();
}

/**
 * Resets the PID controller
*/
void Serializer::ResetPID() {
  m_controller.Reset();
}

/**
 * Sets the PID values
*/
void Serializer::SetPID(double p, double i, double d) {
  m_controller.SetPID(p, i, d);
}

/**
 * Resets everything (call this after arm picks up cone or
 * just as a general reset, such as in TeleopInit or
 * AutonomousInit)
*/
void Serializer::Reset() {
  ResetEncoderValue();
  ResetPID();
  TurnOff();
}