#include "subsystems/Elevator.h"

Elevator::Elevator()
{
    // Starts the configuration process for the first elevator motor
    // This line resets any previous configurations to ensure a clean slate 
    motor1.GetConfigurator().Apply(configs::TalonFXConfiguration{});

    // Sets the motor to brake mode - this is so the elevator stays at the position we tell it to stay at
    motorConfig.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
    // Ensures positive input makes the elevator go up and negative input makes the elevator go down
    motorConfig.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;

    // Stator limit makes sure we don't burn up our motors if they get jammed
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = 120.0_A;

    motorConfig.Slot0.kP = Gains::kP;
    motorConfig.Slot0.kI = Gains::kI;
    motorConfig.Slot0.kD = Gains::kD;
    motorConfig.Slot0.kS = Gains::kS;
    motorConfig.Slot0.kG = Gains::kG;
    motorConfig.Slot0.kV = Gains::kV;
    motorConfig.Slot0.kA = Gains::kA;
    motorConfig.Slot0.GravityType = signals::GravityTypeValue::Elevator_Static;

    motorConfig.MotionMagic.MotionMagicCruiseVelocity = MotionMagic::kCruiseVelocity;
    motorConfig.MotionMagic.MotionMagicAcceleration = MotionMagic::kAcceleration;
    motorConfig.MotionMagic.MotionMagicJerk = MotionMagic::kJerk;

    motor1.GetConfigurator().Apply(motorConfig);

    motor2.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration motor2Config{};

    motor2Config.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;

    motor2Config.CurrentLimits.StatorCurrentLimitEnable = true;
    motor2Config.CurrentLimits.StatorCurrentLimit = 120.0_A;

    motor2.GetConfigurator().Apply(motor2Config);

    // Makes the second elevator motor a follower to the first elevator motor
    motor2.SetControl(follower);

    bottomLimitTrigger.Debounce(100_ms).OnTrue(frc2::cmd::RunOnce([this]
    {
        ResetEncoders();
    }));

    if (frc::RobotBase::IsSimulation())
    {
        topLimitSim.SetValue(false);
        bottomLimitSim.SetValue(true);
    }
}

void Elevator::StopMotors()
{
    motor1.StopMotor(); 
    motor2.StopMotor(); 
}

frc2::CommandPtr Elevator::StopMotorsCommand()
{
    return RunOnce
    (
        [this] 
        { 
            StopMotors();
        }
    ).WithName("ElevatorStopMotor");
}

frc2::CommandPtr Elevator::SetMotorsCommand(double power)
{
    return Run
    (   
        [this, power] 
        {
            motor1.SetControl(controls::DutyCycleOut{power}
                .WithLimitForwardMotion(GetEncoder() > kMaxEncoderValue || IsTopLimitSwitchClosed())
                .WithLimitReverseMotion(IsBottomLimitSwitchClosed()));
        }
    ).Unless
    (
        [this, power] 
        {
            return isElevatorRegistered == false && power > 0;
        }
    ).WithName("ElevatorSetMotors");
}

frc2::CommandPtr Elevator::GoToHeightCommand(units::meter_t desiredHeight)
{
    return StartRun
    (
        [this, desiredHeight]
        {
            this->desiredHeight = desiredHeight;
            StopMotors();
        },
        [this]
        {
            motor1.SetControl(controls::MotionMagicVoltage{(this->desiredHeight - kHeightOffset) / kMetersPerMotorTurn}
                    .WithLimitForwardMotion(GetEncoder() > kMaxEncoderValue || IsTopLimitSwitchClosed())
                    .WithLimitReverseMotion(IsBottomLimitSwitchClosed()));
        }
    ).OnlyIf([this] { return isElevatorRegistered; }).WithName("ElevatorGoToHeight");
}

frc2::CommandPtr Elevator::GoToPositionCommand(const Position &desiredPosition)
{
    return GoToHeightCommand(desiredPosition.height).WithName("ElevatorGoToPosition");
}

void Elevator::InitSendable(wpi::SendableBuilder &builder)
{
    frc2::SubsystemBase::InitSendable(builder);
    
    builder.AddDoubleProperty("desiredHeight",
        [this] { return desiredHeight.value(); },
        {}
    );
    builder.AddDoubleProperty("height",
        [this] { return GetHeight().convert<units::feet>().value(); },
        {}
    );
    builder.AddDoubleProperty("heightSetpoint",
        [this] { return (units::turn_t{motor1.GetClosedLoopReference().GetValueAsDouble()} * kMetersPerMotorTurn + kHeightOffset).convert<units::feet>().value(); },
        {}
    );
    builder.AddBooleanProperty("isAtPosition",
        [this] { return IsAtPosition(); },
        {}
    );
    builder.AddBooleanProperty("bottomLimitSwitch",
        [this] { return IsBottomLimitSwitchClosed(); },
        {}
    );
    builder.AddBooleanProperty("topLimitSwitch",
        [this] { return IsTopLimitSwitchClosed(); },
        {}
    );
    builder.AddBooleanProperty("isElevatorRegistered",
        [this] { return isElevatorRegistered; },
        {}
    );
    builder.AddDoubleProperty("encoder",
        [this] { return GetEncoder().value(); },
        {}
    );
    builder.AddDoubleProperty("encoderSetpoint",
        [this] { return motor1.GetClosedLoopReference().GetValueAsDouble(); },
        {}
    );

    builder.AddDoubleProperty("Gains/kP",
        [this] 
        {
            return motorConfig.Slot0.kP;
        },
        [this](double kP)
        {
            motorConfig.Slot0.kP = kP;
            motor1.GetConfigurator().Apply(motorConfig);
        }
    );
    builder.AddDoubleProperty("Gains/kI",
        [this] 
        {
            return motorConfig.Slot0.kI;
        },
        [this](double kI)
        {
            motorConfig.Slot0.kI = kI;
            motor1.GetConfigurator().Apply(motorConfig);
        }
    );
    builder.AddDoubleProperty("Gains/kD",
        [this] 
        {
            return motorConfig.Slot0.kD;
        },
        [this](double kD)
        {
            motorConfig.Slot0.kD = kD;
            motor1.GetConfigurator().Apply(motorConfig);
        }
    );
    builder.AddDoubleProperty("Gains/kS",
        [this] 
        {
            return motorConfig.Slot0.kS;
        },
        [this](double kS)
        {
            motorConfig.Slot0.kS = kS;
            motor1.GetConfigurator().Apply(motorConfig);
        }
    );
    builder.AddDoubleProperty("Gains/kG",
        [this] 
        {
            return motorConfig.Slot0.kG;
        },
        [this](double kG)
        {
            motorConfig.Slot0.kG = kG;
            motor1.GetConfigurator().Apply(motorConfig);
        }
    );
    builder.AddDoubleProperty("Gains/kV",
        [this] 
        {
            return motorConfig.Slot0.kV;
        },
        [this](double kV)
        {
            motorConfig.Slot0.kV = kV;
            motor1.GetConfigurator().Apply(motorConfig);
        }
    );
    builder.AddDoubleProperty("Gains/kA",
        [this] 
        {
            return motorConfig.Slot0.kA;
        },
        [this](double kA)
        {
            motorConfig.Slot0.kA = kA;
            motor1.GetConfigurator().Apply(motorConfig);
        }
    );
    builder.AddDoubleProperty("MotionMagic/cruiseVelocity",
        [this] 
        { 
            return motorConfig.MotionMagic.MotionMagicCruiseVelocity.value();
        },
        [this](double cruiseVelocity) 
        { 
            motorConfig.MotionMagic.MotionMagicCruiseVelocity = units::turns_per_second_t(cruiseVelocity);
            motor1.GetConfigurator().Apply(motorConfig);
        }
    );
    builder.AddDoubleProperty("MotionMagic/acceleration",
        [this] 
        { 
            return motorConfig.MotionMagic.MotionMagicAcceleration.value();
        },
        [this](double acceleration) 
        { 
            motorConfig.MotionMagic.MotionMagicAcceleration = units::turns_per_second_squared_t(acceleration);
            motor1.GetConfigurator().Apply(motorConfig);
        }
    );
    builder.AddDoubleProperty("MotionMagic/jerk",
        [this] 
        {
            return motorConfig.MotionMagic.MotionMagicJerk.value();
        },
        [this](double jerk) 
        { 
            motorConfig.MotionMagic.MotionMagicJerk = units::turns_per_second_cubed_t(jerk);
            motor1.GetConfigurator().Apply(motorConfig);
        }
    );
}

void Elevator::Periodic()
{
    if (isElevatorRegistered == false && IsBottomLimitSwitchClosed())
    {
        ResetEncoders();
        isElevatorRegistered = true;
    }

    if (frc::RobotBase::IsSimulation())
    {
        ctre::phoenix6::sim::TalonFXSimState& motor1Sim = motor1.GetSimState();
        ctre::phoenix6::sim::TalonFXSimState& motor2Sim = motor2.GetSimState();
        motor1Sim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
        motor2Sim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
        elevatorSim.SetInputVoltage(motor1Sim.GetMotorVoltage());
        elevatorSim.Update(20_ms);

        motor1Sim.SetRawRotorPosition((elevatorSim.GetPosition() - kHeightOffset) / kMetersPerMotorTurn);
        motor2Sim.SetRawRotorPosition((elevatorSim.GetPosition() - kHeightOffset) / kMetersPerMotorTurn);
        motor1Sim.SetRotorVelocity(elevatorSim.GetVelocity() / kMetersPerMotorTurn);
        motor2Sim.SetRotorVelocity(elevatorSim.GetVelocity() / kMetersPerMotorTurn);

        bottomLimitSim.SetValue(elevatorSim.WouldHitLowerLimit(GetHeight()));
    }
}

const units::turn_t Elevator::GetEncoder()
{
    units::turn_t motor1RotorPos = motor1.GetPosition().GetValue();
    units::turn_t motor2RotorPos = motor2.GetPosition().GetValue();
    if (motor1RotorPos >= motor2RotorPos)
    {
        return motor1RotorPos;
    }
    return motor2RotorPos;
}

void Elevator::ResetEncoders()
{
    motor1.SetPosition(0_tr);
    motor2.SetPosition(0_tr);
}