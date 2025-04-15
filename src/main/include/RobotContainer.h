// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>

#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/CommandJoystick.h>

#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/NamedCommands.h>

#include "Constants.h"
#include "subsystems/Drivetrain.h"

class RobotContainer
{
public:
    RobotContainer();

    std::optional<frc2::CommandPtr> GetAutonomousCommand();

private:
    void ConfigureBindings();

    frc2::CommandXboxController gamepad{0};
    frc2::CommandJoystick controlBoard{1};

    ctre::phoenix6::swerve::requests::FieldCentric drive = ctre::phoenix6::swerve::requests::FieldCentric() 
        .WithDeadband(DrivetrainConstants::kMaxSpeed * 0.1).WithRotationalDeadband(DrivetrainConstants::kMaxAngularSpeed * 0.1)
        .WithDriveRequestType(ctre::phoenix6::swerve::impl::DriveRequestType::OpenLoopVoltage);
    ctre::phoenix6::swerve::requests::SwerveDriveBrake brake{};

    Drivetrain swerve{};

    frc::SendableChooser<std::string> autoChooser;
};
