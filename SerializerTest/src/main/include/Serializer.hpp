#include <algorithm> // std::clamp
#include <cmath>     // std::abs

#include <ctre/Phoenix.h>                      // WPI_TalonSRX, PWMChannel
#include <frc/controller/PIDController.h>      // PIDController
#include <frc/DigitalInput.h>                  // DigitalInput
#include <frc/DutyCycleEncoder.h>              // DutyCycleEncoder
#include <frc/fmt/Units.h>                     // units
#include <frc/smartdashboard/SmartDashboard.h> // SmartDashboard

class Serializer {
public:
  Serializer() = delete;
  Serializer(int, int, int, int, double, double, double,
             double, double, double, double, double);

  void RobotInit();
  void AutonomousPeriodic();
  void TeleopPeriodic();
  void DisabledInit();
  void DisabledPeriodic();

  void TurnOn();
  void TurnOff();
  void Reset();
  void ResetEncoderValue();
  void ResetPID();
  void SetPID(double, double, double);
  bool IsPositioned();
private:
  enum States {
    OFF,                // off
    FINDING_CONE,       // spinner go spinny to find cone
    CONE_SWITCHED,      // triggered limit switch once
    CONE_REPOSITIONING, // currently repositioning cone using pid
    CONE_POSITIONED     // cone is repositioned (yay)
  };

  // inputs
  ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_srx;
  frc::DutyCycleEncoder m_encoder;
  frc::DigitalInput m_limitSwitchInput;

  // states
  States m_curState;
  double m_maxVolts;
  double m_curVolts;
  double m_encoderValueWhenHit;
  double m_targetValue;
  double m_angleThresh;
  double m_angleToTarget;

  // pid
  double m_kp, m_ki, m_kd;
  frc2::PIDController m_controller{m_kp, m_ki, m_kd}; 
};