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

    void StopWristMotor()
    {
        wristMotor.StopMotor();
    }

    /// @brief Stops the wrist motor until Set is called
    frc2::CommandPtr StopWristMotorCommand()
    {
        return RunOnce
        (
            [this]
            {
                StopWristMotor();
            }
        );
    }

    /// @brief Manually sets the power to the wrist motor
    /// @param power Power to set to the motor
    frc2::CommandPtr SetWristPowerCommand(double power)
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

    /// @brief Uses PID control to go to a desired angle
    /// @param desiredAngle Desired angle to travel to
    frc2::CommandPtr GoToAngleCommand(units::degree_t desiredAngle)
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
                wristMotor.SetControl(controls::MotionMagicVoltage(desiredAngle)
                                .WithLimitReverseMotion(GetCurrentAngle() >= kHighLimit)
                                .WithLimitForwardMotion(GetCurrentAngle() <= kLowLimit));
            }
        ).WithName("WristGoToAngle");
    }
    /// @brief Sets the angle of the claw to a Position
    /// @param pos Position object
    frc2::CommandPtr GoToPositionCommand(Position desiredPosition)
    {
        return GoToAngleCommand(desiredPosition.angle).WithName("WristGoToPosition");
    }

    /// @brief Gets the current angle of the claw
    /// @return The absolute value of the CANcoder in degrees
    const units::degree_t GetCurrentAngle() { return canCoderWrist.GetAbsolutePosition().GetValue(); }

    bool IsAtPosition() { return units::math::abs<units::degree_t>(GetCurrentAngle() - desiredAngle) < ClawConstants::kTolerance; }

    void InitSendable(wpi::SendableBuilder &builder) override
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
            [this] { return wristMotor.GetClosedLoopReference().GetValue(); },
            {}
        );
    }

    void Periodic() override
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
            wristMotorSim.SetRawRotorPosition(-wristSim.GetAngle() * kWristGearRatio);
            wristMotorSim.SetRotorVelocity(-wristSim.GetVelocity() * kWristGearRatio);
            canCoderSim.SetRawPosition(-wristSim.GetAngle());
            canCoderSim.SetVelocity(-wristSim.GetVelocity());
        }
    }

private:
    // Creates the motors, CANcoder, and proximity sensor
    hardware::TalonFX wristMotor{RobotMap::Claw::kWristMotorID, "rio"};
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
        0_deg,
        180_deg,
        false,
        13.5_deg
    };
};    

class IO : public frc2::SubsystemBase
{
public:
    IO();
    /// @brief Sets the power of the IO (intake/output) motor
    /// @param power Power to set to the motor
    frc2::CommandPtr SetIOPowerCommand(double power)
    {
        return Run
        (
            [this, power]
            {
                ioMotor.SetControl(controls::DutyCycleOut(power));
            }
        ).WithName("IOSetPower");
    }

    frc2::CommandPtr StopIOMotorCommand()
    {
        return RunOnce
        (
            [this] { ioMotor.StopMotor(); }
        );
    }

    /// @brief Gets whether the proximity sensor detects a coral
    /// @return True if the proximity sensor detects a coral, false if not
    bool IsCoralInClaw() { return proxSensor.GetIsDetected().GetValue(); };
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