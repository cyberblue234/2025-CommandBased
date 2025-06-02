#pragma once

#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/configs/Configurator.hpp>

#include <frc/system/plant/DCMotor.h>
#include <frc/simulation/SingleJointedArmSim.h>
#include <frc/system/plant/LinearSystemId.h>

#include "Constants.h"

using namespace WristConstants;
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

    const units::turn_t GetAngleSetpoint() { return units::turn_t{wristMotor.GetClosedLoopReference().GetValue()}; }

    bool IsAtPosition() { return units::math::abs<units::degree_t>(GetCurrentAngle() - desiredAngle) < WristConstants::kTolerance; }

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
            0.05_kg_sq_m,
            kWristGearRatio
        ),
        frc::DCMotor::KrakenX60(1),
        kWristGearRatio,
        1_ft,
        -180_deg,
        180_deg,
        false,
        -48_deg
    };
};    