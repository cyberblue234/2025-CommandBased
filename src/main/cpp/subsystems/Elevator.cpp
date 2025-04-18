#include "subsystems/Elevator.h"

Elevator::Elevator()
{
    // Starts the configuration process for the first elevator motor
    // This line resets any previous configurations to ensure a clean slate 
    motor1.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration motor1Config{};

    // Sets the motor to brake mode - this is so the elevator stays at the position we tell it to stay at
    motor1Config.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
    // Ensures positive input makes the elevator go up and negative input makes the elevator go down
    motor1Config.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;

    // Stator limit makes sure we don't burn up our motors if they get jammed
    motor1Config.CurrentLimits.StatorCurrentLimitEnable = true;
    motor1Config.CurrentLimits.StatorCurrentLimit = 120.0_A;

    motor1Config.Slot0.kP = kP;
    motor1Config.Slot0.kI = kI;
    motor1Config.Slot0.kD = kD;
    motor1Config.Slot0.kS = kS;
    motor1Config.Slot0.kG = kG;
    motor1Config.Slot0.kV = kV;
    motor1Config.Slot0.kA = kA;
    motor1Config.Slot0.GravityType = signals::GravityTypeValue::Elevator_Static;

    motor1Config.MotionMagic.MotionMagicCruiseVelocity = kCruiseVelocity;
    motor1Config.MotionMagic.MotionMagicAcceleration = kAcceleration;
    motor1Config.MotionMagic.MotionMagicJerk = kJerk;

    motor1.GetConfigurator().Apply(motor1Config);

    motor2.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration motor2Config{};

    motor2Config.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;

    motor2Config.CurrentLimits.StatorCurrentLimitEnable = true;
    motor2Config.CurrentLimits.StatorCurrentLimit = 120.0_A;

    motor2.GetConfigurator().Apply(motor2Config);

    // Makes the second elevator motor a follower to the first elevator motor
    motor2.SetControl(follower);

    if (frc::RobotBase::IsSimulation())
    {
        topLimitSim.SetValue(false);
        bottomLimitSim.SetValue(true);
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