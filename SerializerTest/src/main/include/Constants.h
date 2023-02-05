namespace Constants {
  // pins and IDs
  const unsigned int SERIALIZER_MOTOR_ID = 41;
  const unsigned int SERIALIZER_ENCODER_INPUT_PIN = 0;
  const unsigned int SERIALIZER_LIMIT_SWITCH_PIN = 1;

  // voltages
  const unsigned int SERIALIZER_MAXIMUM_VOLTAGE = 2;
  const int SERIALIZER_DEFAULT_VOLTAGE = -2;
  
  // angles
  const double SERIALIZER_CONE_ANGLE_THRESH = 0.25; // angle the turntable needs to rotate to get from one
                                                    // side of the base of the cone to the other.
                                                    // to measure this, get the encoder value from "Serializer Encoder Value"
                                                    // when one side of the cone triggers the limit switch (and the limit switch
                                                    // is outside the base), turn the cone until the other side of the base
                                                    // touches the triggers switch, get the encoder value, and subtract the two
                                                    // values. This can also be changed with "Serializer Angle Threshold" on smartdashboard.
  const double SERIALIZER_ANG_TO_TARGET = -0.55;    // angle cone needs to turn from the inside edge/hole touching the limit switch to the
                                                    // target position, measured in encoder values (look at the "Serializer Encoder
                                                    // Value") on smartdashboard. This can also be changed with "Serializer Target Ang"
                                                    // on smartdashboard.

  // PID
  const double SERIALIZER_KP = 7;
  const double SERIALIZER_KI = 5;
  const double SERIALIZER_KD = 2.7;
  const double SERIALIZER_ERR_TOL = 0.01;     // error tolerance
  const double SERIALIZER_ERR_DER_TOL = 1; // error derivative tolerance
} // namespace Constants