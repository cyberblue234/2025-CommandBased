#pragma once

#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANrange.hpp>
#include <ctre/phoenix6/configs/Configurator.hpp>

#include <frc/system/plant/DCMotor.h>
#include <frc/simulation/FlywheelSim.h>
#include <frc/system/plant/LinearSystemId.h>

#include "Constants.h"

using namespace IOConstants;
using namespace ctre::phoenix6;

class IO : public frc2::SubsystemBase
{
public:
    IO();
    void SetIOPower(double power);
    void StopIOMotor();
    /// @brief Sets the power of the IO (intake/output) motor
    /// @param power Power to set to the motor
    frc2::CommandPtr SetIOPowerCommand(double power);
    frc2::CommandPtr StopIOMotorCommand();
    frc2::CommandPtr IOAtPosition(std::function<const Position()> positionSupplier);

    /// @brief Gets whether the proximity sensor detects a coral
    /// @return True if the proximity sensor detects a coral, false if not
    bool IsCoralInClaw() { return proxSensor.GetIsDetected().GetValue(); };
    /// @brief Gets the distance to the closest object from the proximity sensor
    /// @return Distance from the closest object
    units::meter_t GetDistance() { return proxSensor.GetDistance().GetValue(); }

    units::turns_per_second_t GetMotorVelocity() { return ioMotor.GetVelocity().GetValue(); }

    void InitSendable(wpi::SendableBuilder &builder) override
    {
        frc2::SubsystemBase::InitSendable(builder);

        builder.AddBooleanProperty("coralInClaw",
            [this] { return IsCoralInClaw(); },
            [this] (bool setCoralInClaw)
            {
                if (frc::RobotBase::IsSimulation())
                {
                    SetSimProximitySensorDistance(setCoralInClaw ? 0_m : kProximityThreshold + 1_in);
                }
            }
        );
        builder.AddDoubleProperty("distance",
            [this] { return GetDistance().convert<units::inch>().value(); },
            {}
        );
        builder.AddDoubleProperty("speed",
            [this] { return ioMotor.Get(); },
            {}
        );
    }

    void Periodic() override
    {
        if (frc::RobotBase::IsSimulation())
        {
            ctre::phoenix6::sim::TalonFXSimState& ioMotorSim = ioMotor.GetSimState();
            ioMotorSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
            ioSim.SetInputVoltage(ioMotorSim.GetMotorVoltage());
            ioSim.Update(20_ms);
            ioMotorSim.AddRotorPosition(ioSim.GetAngularVelocity() * 0.02_s * kIOGearRatio);
            ioMotorSim.SetRotorVelocity(ioSim.GetAngularVelocity() * kIOGearRatio);
            ioMotorSim.SetRotorAcceleration(ioSim.GetAngularAcceleration() * kIOGearRatio);
        }
    }

    void SetSimProximitySensorDistance(units::meter_t dist)
    {
        sim::CANrangeSimState &proxSim = proxSensor.GetSimState();
		proxSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
		proxSim.SetDistance(dist);
    }

    hardware::CANrange &GetProximitySensor() { return proxSensor; }
    hardware::TalonFX &GetIOMotor() { return ioMotor; }
private:
    hardware::CANrange proxSensor{RobotMap::Claw::kCanRangeID, "rio"};
    hardware::TalonFX ioMotor{RobotMap::Claw::kIOMotorID, "rio"};

    frc::sim::FlywheelSim ioSim
    {
        frc::LinearSystemId::FlywheelSystem
        (
            frc::DCMotor::KrakenX60(1),
            units::kilogram_square_meter_t{0.001},
            kIOGearRatio
        ),
        frc::DCMotor::KrakenX60(1)
    };
};