#pragma once

#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/CANrange.hpp>
#include <ctre/phoenix6/configs/Configurator.hpp>

#include <frc/system/plant/DCMotor.h>
#include <frc/simulation/SingleJointedArmSim.h>
#include <frc/system/plant/LinearSystemId.h>

#include "Constants.h"

using namespace ClawConstants;
using namespace ctre::phoenix6;

class Wrist : public frc2::SubsystemBase
{
public:
    Wrist();

    void StopWristMotor();

    /// @brief Stops the wrist motor until Set is called
    frc2::CommandPtr StopWristMotorCommand();

    /// @brief Manually sets the power to the wrist motor
    /// @param power Power to set to the motor
    frc2::CommandPtr SetWristPowerCommand(double power);

    /// @brief Uses PID control to go to a desired angle
    /// @param desiredAngle Desired angle to travel to
    frc2::CommandPtr GoToAngleCommand(units::degree_t desiredAngle);
    /// @brief Sets the angle of the claw to a Position
    /// @param pos Position object
    frc2::CommandPtr GoToPositionCommand(const Position &desiredPosition);

    /// @brief Gets the current angle of the claw
    /// @return The absolute value of the CANcoder in degrees
    const units::degree_t GetCurrentAngle() { return canCoderWrist.GetAbsolutePosition().GetValue(); }

    bool IsAtPosition() { return units::math::abs<units::degree_t>(GetCurrentAngle() - desiredAngle) < ClawConstants::kTolerance; }

    void InitSendable(wpi::SendableBuilder &builder) override;

    void Periodic() override;

private:
    // Creates the motors, CANcoder, and proximity sensor
    hardware::TalonFX wristMotor{RobotMap::Claw::kWristMotorID, "rio"};
    configs::TalonFXConfiguration wristMotorConfig{};
    hardware::CANcoder canCoderWrist{RobotMap::Claw::kCanCoderID,"rio"};

    units::degree_t desiredAngle;

    frc::sim::SingleJointedArmSim wristSim
    {
        frc::LinearSystemId::SingleJointedArmSystem
        (
            frc::DCMotor::KrakenX60(1),
            1.0_kg_sq_m,
            kWristGearRatio
        ),
        frc::DCMotor::KrakenX60(1),
        kWristGearRatio,
        1_ft,
        -10_deg,
        180_deg,
        false,
        13.5_deg
    };
};    

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
    frc2::CommandPtr IOAtPosition(std::function<const Position*()> positionSupplier);

    /// @brief Gets whether the proximity sensor detects a coral
    /// @return True if the proximity sensor detects a coral, false if not
    bool IsCoralInClaw() { return frc::RobotBase::IsReal() ? proxSensor.GetIsDetected().GetValue() : frc::SmartDashboard::GetBoolean("IO/SimCoralInClaw", false); };
    /// @brief Gets the distance to the closest object from the proximity sensor
    /// @return Distance from the closest object
    units::meter_t GetDistance() { return proxSensor.GetDistance().GetValue(); }

    void InitSendable(wpi::SendableBuilder &builder) override
    {
        frc2::SubsystemBase::InitSendable(builder);
    }
private:
    hardware::CANrange proxSensor{RobotMap::Claw::kCanRangeID, "rio"};
    hardware::TalonFX ioMotor{RobotMap::Claw::kIOMotorID, "rio"};
};