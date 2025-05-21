#include "subsystems/IO.h"

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
    proxSensorConfig.ProximityParams.ProximityThreshold = kProximityThreshold;

    proxSensor.GetConfigurator().Apply(proxSensorConfig);
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

frc2::CommandPtr IO::IOAtPosition(std::function<const Position()> positionSupplier)
{
    return Run
    (
        [this, positionSupplier]
        {
            SetIOPower(positionSupplier().ioMotorPower);
        }
    ).Until
    (
        [this, positionSupplier]
        {
            if (positionSupplier().isForCoral == false) return false;
            return IsCoralInClaw() == positionSupplier().isForIntake;
        }
    ).AndThen
    (
        [this] { StopIOMotor(); }
    ).WithName("IOAtPosition");
}