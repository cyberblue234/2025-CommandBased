#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/button/Trigger.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configurator.hpp>
#include <ctre/phoenix6/controls/Follower.hpp>

#include <ctre/phoenix6/sim/CANcoderSimState.hpp>
#include <ctre/phoenix6/sim/TalonFXSimState.hpp>

#include <frc/system/plant/DCMotor.h>
#include <frc/simulation/ElevatorSim.h>
#include <frc/system/plant/LinearSystemId.h>

#include <frc/DigitalInput.h>

#include <frc/Timer.h>

#include "Constants.h"

using namespace ElevatorConstants;
using namespace ctre::phoenix6;

class Elevator : public frc2::SubsystemBase
{
public:
    /// @brief Constructs the elevator
    Elevator();

    void StopMotors()
    {
        motor1.StopMotor(); 
        motor2.StopMotor(); 
    }

    frc2::CommandPtr StopMotorsCommand()
    {
        return RunOnce
        (
            [this] 
            { 
                StopMotors();
            }
        );
    }

    frc2::CommandPtr SetMotorsCommand(double power)
    {
        return Run
        (   
            [this, power] 
            {
                motor1.SetControl(controls::DutyCycleOut{power}
                    .WithLimitForwardMotion(GetEncoder() > kMaxEncoderValue || IsTopLimitSwitchClosed())
                    .WithLimitReverseMotion(IsBottomLimitSwitchClosed()));
                if (frc::RobotBase::IsSimulation())
                {
                    ctre::phoenix6::sim::TalonFXSimState& motor1Sim = motor1.GetSimState();
                    ctre::phoenix6::sim::TalonFXSimState& motor2Sim = motor2.GetSimState();
                    motor1Sim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
                    motor2Sim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
                    motor1Sim.AddRotorPosition((1_mps / kMetersPerMotorTurn * power) * 0.02_s);
                    motor2Sim.AddRotorPosition((1_mps / kMetersPerMotorTurn * power) * 0.02_s);
                }
            }
        ).Unless
        (
            [this, power] 
            {
                return isElevatorRegistered == false && power > 0;
            }
        );
    }

    frc2::CommandPtr GoToHeightCommand(units::meter_t desiredHeight)
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
                if (frc::RobotBase::IsSimulation())
                {
                    ctre::phoenix6::sim::TalonFXSimState& motor1Sim = motor1.GetSimState();
                    ctre::phoenix6::sim::TalonFXSimState& motor2Sim = motor2.GetSimState();
                    motor1Sim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
                    motor2Sim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
                    units::turn_t setpoint{motor1.GetClosedLoopReference().GetValue()};
                    motor1Sim.SetRawRotorPosition(setpoint);
                    motor2Sim.SetRawRotorPosition(setpoint);
                }
            }
        );
    }

    frc2::CommandPtr GoToPositionCommand(Position desiredPosition)
    {
        return GoToHeightCommand(desiredPosition.height).BeforeStarting([this, desiredPosition]
        {
            this->desiredPosition = desiredPosition;
        });
    }
    
    /// @brief Gets the greater of the two motors' encoder values
    /// @return The greater encoder value
    const units::turn_t GetEncoder();
    /// @brief Gets the physical height of the elevator based on encoder values
    /// @return The height of the elevator
    const units::meter_t GetHeight() { return GetEncoder() * kMetersPerMotorTurn + kHeightOffset; }
    /// @brief Sets both encoders to 0
    void ResetEncoders();

    /// @brief Gets the state of the bottom limit switch
    /// @retval true if the limit switch is closed (pressed)
    /// @retval false if the limit switch is open
    bool IsBottomLimitSwitchClosed() { return bottomLimitSwitch.Get() && simLimSwitch; }

    /// @brief Gets the state of the top limit switch
    /// @retval true if the limit switch is closed (pressed)
    /// @retval false if the limit switch is open
    bool IsTopLimitSwitchClosed() 
    {
        if (bypassTopLimit) return false;
        return topLimitSwitch.Get(); 
    }

    bool IsAtPosition() { return units::math::abs<units::meter_t>(GetHeight() - desiredHeight) < kTolerance; }

    void SetBypassTopLimit(bool set)
    {
        bypassTopLimit = set;
    }

    void InitSendable(wpi::SendableBuilder &builder) override
    {
        frc2::SubsystemBase::InitSendable(builder);

        builder.AddDoubleProperty("height",
            [this] { return GetHeight().convert<units::feet>().value(); },
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
    }

    void Periodic() override 
    {
        stage1Publisher.Set(frc::Pose3d{0_m, 0_m, (GetHeight() - kHeightOffset) / 2, frc::Rotation3d{0_deg, 0_deg, 0_deg}});
        carriagePublisher.Set(frc::Pose3d{0_m, 0_m, GetHeight() - kHeightOffset, frc::Rotation3d{0_deg, 0_deg, 0_deg}});
    }

private:
    // Creates the motor objects
    hardware::TalonFX motor1{RobotMap::Elevator::kMotor1ID, "rio"};
    hardware::TalonFX motor2{RobotMap::Elevator::kMotor2ID, "rio"};

    units::meter_t desiredHeight;
    Position desiredPosition;

    // Creates the limit switch - it is a digital (true or false) input
    frc::DigitalInput bottomLimitSwitch{RobotMap::Elevator::kBottomLimitSwitchID};
    frc2::Trigger bottomLimitTrigger{[this] { return IsBottomLimitSwitchClosed(); }};
    frc::DigitalInput topLimitSwitch{RobotMap::Elevator::kTopLimitSwitchID};
    bool bypassTopLimit = false;
    // Simulated representation of the limit switch
    bool simLimSwitch = true;

    // If the elevator is not registered, the elevator encoders have not been reset. 
    // In that case, we will not allow the elevator to use PID control
    bool isElevatorRegistered = false;

    // Whether or not to output the negative input to the follower motor
    bool followerInverted = false;
    // Will use this to set motor2 to a follower motor of motor1
    // This means every input to motor1 will be sent to motor2
    // If followerInverted is true, the input will be the opposite (5 volts -> -5 volts)
    controls::Follower follower{RobotMap::Elevator::kMotor1ID, followerInverted};

    nt::StructPublisher<frc::Pose3d> stage1Publisher = nt::NetworkTableInstance::GetDefault().GetTable("Robot")->GetStructTopic<frc::Pose3d>("stage_1").Publish();
    nt::StructPublisher<frc::Pose3d> carriagePublisher = nt::NetworkTableInstance::GetDefault().GetTable("Robot")->GetStructTopic<frc::Pose3d>("carriage").Publish();
    
};
