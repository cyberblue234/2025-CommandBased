#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/button/Trigger.h>
#include <frc/filter/Debouncer.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configurator.hpp>
#include <ctre/phoenix6/controls/Follower.hpp>

#include <ctre/phoenix6/sim/CANcoderSimState.hpp>
#include <ctre/phoenix6/sim/TalonFXSimState.hpp>

#include <frc/system/plant/DCMotor.h>
#include <frc/simulation/ElevatorSim.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/simulation/DIOSim.h>

#include <frc/DigitalInput.h>

#include <frc/Timer.h>

#include <unordered_map>

#include "Constants.h"

using namespace ElevatorConstants;
using namespace ctre::phoenix6;

class Elevator : public frc2::SubsystemBase
{
public:
    /// @brief Constructs the elevator
    Elevator();

    void StopMotors();

    frc2::CommandPtr StopMotorsCommand();

    frc2::CommandPtr SetMotorsCommand(double power);

    frc2::CommandPtr GoToHeightCommand(units::meter_t desiredHeight);

    frc2::CommandPtr GoToPositionCommand(const Position &desiredPosition);
    
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
    bool IsBottomLimitSwitchClosed() { return bottomLimitSwitch.Get(); }

    /// @brief Gets the state of the top limit switch
    /// @retval true if the limit switch is closed (pressed)
    /// @retval false if the limit switch is open
    bool IsTopLimitSwitchClosed() 
    {
        if (bypassTopLimit) return false;
        return topLimitSwitch.Get(); 
    }

    bool IsAtPosition() { return units::math::abs<units::meter_t>(GetHeight() - desiredHeight) < ElevatorConstants::kTolerance; }

    void SetBypassTopLimit(bool set)
    {
        bypassTopLimit = set;
    }

    void InitSendable(wpi::SendableBuilder &builder) override;

    void Periodic() override;

    hardware::TalonFX *GetMotor1() { return &motor1; }
    hardware::TalonFX *GetMotor2() { return &motor2; }

private:
    // Creates the motor objects
    hardware::TalonFX motor1{RobotMap::Elevator::kMotor1ID, "rio"};
    hardware::TalonFX motor2{RobotMap::Elevator::kMotor2ID, "rio"};

    configs::TalonFXConfiguration motorConfig{};

    units::meter_t desiredHeight;

    // Creates the limit switch - it is a digital (true or false) input
    frc::DigitalInput bottomLimitSwitch{RobotMap::Elevator::kBottomLimitSwitchID};
    frc2::Trigger bottomLimitTrigger{[this] { return IsBottomLimitSwitchClosed(); }};
    frc::DigitalInput topLimitSwitch{RobotMap::Elevator::kTopLimitSwitchID};
    bool bypassTopLimit = false;
    // Simulated representation of the limit switch
    frc::sim::DIOSim bottomLimitSim{bottomLimitSwitch};
    frc::sim::DIOSim topLimitSim{topLimitSwitch};

    // If the elevator is not registered, the elevator encoders have not been reset. 
    // In that case, we will not allow the elevator to use PID control
    bool isElevatorRegistered = false;

    // Whether or not to output the negative input to the follower motor
    bool followerInverted = false;
    // Will use this to set motor2 to a follower motor of motor1
    // This means every input to motor1 will be sent to motor2
    // If followerInverted is true, the input will be the opposite (5 volts -> -5 volts)
    controls::Follower follower{RobotMap::Elevator::kMotor1ID, followerInverted};

    

    frc::sim::ElevatorSim elevatorSim
    {
        frc::DCMotor::KrakenX60(2),
        kMotorGearing,
        20_lb,
        kSprocketPitchDiameter,
        kHeightOffset,
        kMaxElevatorHeight,
        false,
        kHeightOffset
    };
    
};
