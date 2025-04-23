#pragma once

#include <frc2/command/SubsystemBase.h>

#include "rev/SparkFlex.h"
#include <frc/DigitalInput.h>

#include "Constants.h"

using namespace ClimberConstants;
using namespace rev::spark;

class Climber : public frc2::SubsystemBase
{
public:
    /// @brief Constructs the climber
    Climber();

    /// @brief Manually sets the power of the climber motor
    /// @param power Power to set the motor to
    void SetPower(double power);
    frc2::CommandPtr SetPowerCommand(double power);
    frc2::CommandPtr StopMotorCommand();

    bool GetLimit(){ return limit.Get(); }

    void InitSendable(wpi::SendableBuilder &builder) override;
private:
    // Creates the climber motor
    SparkFlex climbMotor{RobotMap::Climber::kClimbMotorID, SparkLowLevel::MotorType::kBrushless};

    frc::DigitalInput limit{RobotMap::Climber::kLimitSwitch};
};