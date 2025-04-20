#include "subsystems/Claw.h"

Wrist::Wrist()
{
    // Starts the configuration process for the wrist motor
    // This line resets any previous configurations to ensure a clean slate
    wristMotor.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration wristMotorConfig{};

    // Stops the motor if there is no input - desirable for ensuring the wrist stays at the desired position
    wristMotorConfig.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
    wristMotorConfig.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;

    wristMotorConfig.Slot0.kP = Gains::kP;
    wristMotorConfig.Slot0.kI = Gains::kI;
    wristMotorConfig.Slot0.kD = Gains::kD;
    wristMotorConfig.Slot0.kS = Gains::kS;
    wristMotorConfig.Slot0.kG = Gains::kG;
    wristMotorConfig.Slot0.kV = Gains::kV;
    wristMotorConfig.Slot0.kA = Gains::kA;
    wristMotorConfig.Slot0.GravityType = signals::GravityTypeValue::Arm_Cosine;

    wristMotorConfig.MotionMagic.MotionMagicCruiseVelocity = MotionMagic::kCruiseVelocity;
    wristMotorConfig.MotionMagic.MotionMagicAcceleration = MotionMagic::kAcceleration;
    wristMotorConfig.MotionMagic.MotionMagicJerk = MotionMagic::kJerk;

    // Stator limit makes sure we don't burn up our motors if they get jammed
    wristMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    wristMotorConfig.CurrentLimits.StatorCurrentLimit = 120.0_A;

    // Sets the CANcoder as the encoder for the wrist motor
    wristMotorConfig.Feedback.FeedbackRemoteSensorID = canCoderWrist.GetDeviceID();
    wristMotorConfig.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::RemoteCANcoder;

    // Applies the configuration
    wristMotor.GetConfigurator().Apply(wristMotorConfig);

    canCoderWrist.GetConfigurator().Apply(configs::CANcoderConfiguration{});
    configs::CANcoderConfiguration canCoderWristConfig{};

    // Sets the offset for the CANcoder - makes sure 0 is when the wrist is horizontal
    if (frc::RobotBase::IsReal()) canCoderWristConfig.MagnetSensor.MagnetOffset = kCanCoderMagnetOffset;
    // Sets the range of the CANcoder. When it is at 0.5 turn, the CANcoders range is from [-0.2, 0.8)
    canCoderWristConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.8_tr;
    canCoderWristConfig.MagnetSensor.SensorDirection = signals::SensorDirectionValue::Clockwise_Positive;

    canCoderWrist.GetConfigurator().Apply(canCoderWristConfig);
}


IO::IO()
{
    // This line resets any previous configurations to ensure a clean slate
    ioMotor.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration ioMotorConfig{};

    // Stops the motor if there is no input - desirable for ensuring the wrist stays at the desired position
    ioMotorConfig.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;

    // Stator limit makes sure we don't burn up our motors if they get jammed
    ioMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    ioMotorConfig.CurrentLimits.StatorCurrentLimit = 120.0_A;

    // Applies the configuration
    ioMotor.GetConfigurator().Apply(ioMotorConfig);

    proxSensor.GetConfigurator().Apply(configs::CANrangeConfiguration{});
    configs::CANrangeConfiguration proxSensorConfig{};

    // If an object is detected by the proximity sensor within this value, the .GetIsDetected() will return true
    proxSensorConfig.ProximityParams.ProximityThreshold = 2_in;

    proxSensor.GetConfigurator().Apply(proxSensorConfig);
}