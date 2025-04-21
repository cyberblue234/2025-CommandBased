#include "subsystems/Claw.h"

Wrist::Wrist()
{
    // Starts the configuration process for the wrist motor
    // This line resets any previous configurations to ensure a clean slate
    wristMotor.GetConfigurator().Apply(configs::TalonFXConfiguration{});

    // Stops the motor if there is no input - desirable for ensuring the wrist stays at the desired position
    wristMotorConfig.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
    if (frc::RobotBase::IsReal()) wristMotorConfig.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;

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
    if (frc::RobotBase::IsReal()) canCoderWristConfig.MagnetSensor.SensorDirection = signals::SensorDirectionValue::Clockwise_Positive;

    canCoderWrist.GetConfigurator().Apply(canCoderWristConfig);
}

void Wrist::StopWristMotor()
{
    wristMotor.StopMotor();
}

frc2::CommandPtr Wrist::StopWristMotorCommand()
{
    return RunOnce
    (
        [this]
        {
            StopWristMotor();
        }
    );
}

frc2::CommandPtr Wrist::SetWristPowerCommand(double power)
{
    return Run
    (
        [this, power]
        {
            wristMotor.SetControl(controls::DutyCycleOut{power}
                        .WithLimitForwardMotion(GetCurrentAngle() <= 1_deg));
        }
    );
}

frc2::CommandPtr Wrist::GoToAngleCommand(units::degree_t desiredAngle)
{
    return StartRun
    (
        [this, desiredAngle]
        {
            this->desiredAngle = desiredAngle;
            StopWristMotor();
        },
        [this, desiredAngle]
        {
            wristMotor.SetControl(controls::MotionMagicVoltage{desiredAngle}
                            .WithLimitReverseMotion(frc::RobotBase::IsReal() ? GetCurrentAngle() >= kHighLimit : GetCurrentAngle() <= kLowLimit)
                            .WithLimitForwardMotion(frc::RobotBase::IsReal() ? GetCurrentAngle() <= kLowLimit : GetCurrentAngle() >= kHighLimit));
            frc::SmartDashboard::PutNumber("Wrist/test", wristMotor.GetClosedLoopError().GetValue());
        }
    ).WithName("WristGoToAngle");
}

frc2::CommandPtr Wrist::GoToPositionCommand(const Position &desiredPosition)
{
    return GoToAngleCommand(desiredPosition.angle).WithName("WristGoToPosition");
}

void Wrist::InitSendable(wpi::SendableBuilder &builder)
{
    frc2::SubsystemBase::InitSendable(builder);
    
    builder.AddDoubleProperty("desiredAngle",
        [this] { return desiredAngle.value(); },
        {}
    );
    builder.AddDoubleProperty("angle",
        [this] { return GetCurrentAngle().value(); },
        {}
    );
    builder.AddDoubleProperty("angleSetpoint",
        [this] { return wristMotor.GetClosedLoopReference().GetValue() * 360; },
        {}
    );
    builder.AddDoubleProperty("turns",
        [this] { return canCoderWrist.GetAbsolutePosition().GetValueAsDouble(); },
        {}
    );
    builder.AddDoubleProperty("turnSetpoint",
        [this] { return wristMotor.GetClosedLoopReference().GetValue(); },
        {}
    );
    builder.AddBooleanProperty("isAtPosition",
        [this] { return IsAtPosition(); },
        {}
    );
    builder.AddDoubleProperty("Gains/kP",
    [this] 
    {
        return wristMotorConfig.Slot0.kP;
    },
    [this](double kP)
    {
        wristMotorConfig.Slot0.kP = kP;
        wristMotor.GetConfigurator().Apply(wristMotorConfig);
    }
    );
    builder.AddDoubleProperty("Gains/kI",
        [this] 
        {
            return wristMotorConfig.Slot0.kI;
        },
        [this](double kI)
        {
            wristMotorConfig.Slot0.kI = kI;
            wristMotor.GetConfigurator().Apply(wristMotorConfig);
        }
    );
    builder.AddDoubleProperty("Gains/kD",
        [this] 
        {
            return wristMotorConfig.Slot0.kD;
        },
        [this](double kD)
        {
            wristMotorConfig.Slot0.kD = kD;
            wristMotor.GetConfigurator().Apply(wristMotorConfig);
        }
    );
    builder.AddDoubleProperty("Gains/kS",
        [this] 
        {
            return wristMotorConfig.Slot0.kS;
        },
        [this](double kS)
        {
            wristMotorConfig.Slot0.kS = kS;
            wristMotor.GetConfigurator().Apply(wristMotorConfig);
        }
    );
    builder.AddDoubleProperty("Gains/kG",
        [this] 
        {
            return wristMotorConfig.Slot0.kG;
        },
        [this](double kG)
        {
            wristMotorConfig.Slot0.kG = kG;
            wristMotor.GetConfigurator().Apply(wristMotorConfig);
        }
    );
    builder.AddDoubleProperty("Gains/kV",
        [this] 
        {
            return wristMotorConfig.Slot0.kV;
        },
        [this](double kV)
        {
            wristMotorConfig.Slot0.kV = kV;
            wristMotor.GetConfigurator().Apply(wristMotorConfig);
        }
    );
    builder.AddDoubleProperty("Gains/kA",
        [this] 
        {
            return wristMotorConfig.Slot0.kA;
        },
        [this](double kA)
        {
            wristMotorConfig.Slot0.kA = kA;
            wristMotor.GetConfigurator().Apply(wristMotorConfig);
        }
    );
    builder.AddDoubleProperty("MotionMagic/cruiseVelocity",
        [this] 
        { 
            return wristMotorConfig.MotionMagic.MotionMagicCruiseVelocity.value();
        },
        [this](double cruiseVelocity) 
        { 
            wristMotorConfig.MotionMagic.MotionMagicCruiseVelocity = units::turns_per_second_t(cruiseVelocity);
            wristMotor.GetConfigurator().Apply(wristMotorConfig);
        }
    );
    builder.AddDoubleProperty("MotionMagic/acceleration",
        [this] 
        { 
            return wristMotorConfig.MotionMagic.MotionMagicAcceleration.value();
        },
        [this](double acceleration) 
        { 
            wristMotorConfig.MotionMagic.MotionMagicAcceleration = units::turns_per_second_squared_t(acceleration);
            wristMotor.GetConfigurator().Apply(wristMotorConfig);
        }
    );
    builder.AddDoubleProperty("MotionMagic/jerk",
        [this] 
        {
            return wristMotorConfig.MotionMagic.MotionMagicJerk.value();
        },
        [this](double jerk) 
        { 
            wristMotorConfig.MotionMagic.MotionMagicJerk = units::turns_per_second_cubed_t(jerk);
            wristMotor.GetConfigurator().Apply(wristMotorConfig);
        }
    );
}

void Wrist::Periodic()
{
    if (frc::RobotBase::IsSimulation())
    {
        ctre::phoenix6::sim::TalonFXSimState& wristMotorSim = wristMotor.GetSimState();
        ctre::phoenix6::sim::CANcoderSimState& canCoderSim = canCoderWrist.GetSimState();
        wristMotorSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
        canCoderSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
        wristSim.SetInputVoltage(wristMotorSim.GetMotorVoltage());
        wristSim.Update(20_ms);
        frc::SmartDashboard::PutNumber("Wrist/simAngle", wristSim.GetAngle().convert<units::degree>().value());
        wristMotorSim.SetRawRotorPosition(wristSim.GetAngle() * kWristGearRatio);
        wristMotorSim.SetRotorVelocity(wristSim.GetVelocity() * kWristGearRatio);
        canCoderSim.SetRawPosition(wristSim.GetAngle());
        canCoderSim.SetVelocity(wristSim.GetVelocity());
    }
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

    frc::SmartDashboard::PutBoolean("IO/SimCoralInClaw", false);
}

void IO::SetIOPower(double power)
{
    ioMotor.SetControl(controls::DutyCycleOut(power));
}

void IO::StopIOMotor()
{
    ioMotor.StopMotor();
}

frc2::CommandPtr IO::SetIOPowerCommand(double power)
{
    return Run
    (
        [this, power]
        {
            SetIOPower(power);
        }
    ).WithName("IOSetPower");
}

frc2::CommandPtr IO::StopIOMotorCommand()
{
    return RunOnce
    (
        [this] { StopIOMotor(); }
    ).WithName("IOStopMotor");
}

frc2::CommandPtr IO::IOAtPosition(std::function<const Position*()> positionSupplier)
{
    return Run
    (
        [this, positionSupplier]
        {
            SetIOPower(positionSupplier()->ioMotorPower);
        }
    ).Until
    (
        [this, positionSupplier]
        {
            return IsCoralInClaw() == positionSupplier()->isForCoralIntake;
        }
    ).AndThen
    (
        [this] { StopIOMotor(); }
    ).WithName("IOAtPosition");
}