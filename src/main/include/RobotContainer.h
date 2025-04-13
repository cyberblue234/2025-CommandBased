// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>

#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/CommandJoystick.h>

#include "Constants.h"
#include "subsystems/Drivetrain.h"

class RobotContainer
{
public:
    RobotContainer();

    frc2::CommandPtr GetAutonomousCommand();

private:
    void ConfigureBindings();

    frc2::CommandXboxController gamepad{0};
    frc2::CommandJoystick controlBoard{1};

    ctre::phoenix6::swerve::requests::FieldCentric drive = ctre::phoenix6::swerve::requests::FieldCentric() 
        .WithDeadband(units::meters_per_second_t{DrivetrainConstants::kMaxSpeed.value() * 0.1}).WithRotationalDeadband(units::radians_per_second_t{DrivetrainConstants::kMaxAngularSpeed.value() * 0.1})
        .WithDriveRequestType(ctre::phoenix6::swerve::impl::DriveRequestType::OpenLoopVoltage);
    ctre::phoenix6::swerve::requests::SwerveDriveBrake brake{};

    Drivetrain swerve{};
};
