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
#include "subsystems/Elevator.h"
#include "subsystems/Claw.h"
#include "subsystems/Climber.h"

#include <stdio.h>

class RobotContainer
{
public:
    RobotContainer();

    std::optional<frc2::CommandPtr> GetAutonomousCommand();

    Drivetrain *GetSwerve() { return &swerve; }
    Elevator *GetElevator() { return &elevator; }
    Wrist *GetWrist() { return &wrist; }
    IO *GetIO() { return &io; }
    Climber *GetClimber() { return &climber; }

    frc2::CommandPtr GoToPositionCommand(const Position &desiredPosition)
    {
        return frc2::cmd::Parallel
        (
            elevator.GoToPositionCommand(desiredPosition),
            wrist.GoToPositionCommand(desiredPosition)
        ).BeforeStarting
        (
            [this, desiredPosition]
            {
                this->desiredPosition = &desiredPosition;
            }
        );
    }

    void UpdateSimulatedRobotComponents()
    {
        stage1Publisher.Set(frc::Pose3d{0_m, 0_m, (elevator.GetHeight() - ElevatorConstants::kHeightOffset) / 2, frc::Rotation3d{0_deg, 0_deg, 0_deg}});
        carriagePublisher.Set(frc::Pose3d{0_m, 0_m, elevator.GetHeight() - ElevatorConstants::kHeightOffset, frc::Rotation3d{0_deg, 0_deg, 0_deg}});
        clawPublisher.Set(frc::Pose3d{0.265_m, 0_m, elevator.GetHeight() - ElevatorConstants::kHeightOffset + 0.4354_m, frc::Rotation3d{0_deg, wrist.GetCurrentAngle(), 0_deg}});
    }

    void UpdateTelemetry()
    {
        frc::SmartDashboard::PutString("Robot/desiredPosition", desiredPosition ? desiredPosition->to_string() : "null");
    }

private:
    void ConfigureBindings();

    frc2::CommandXboxController gamepad{0};
    frc2::CommandJoystick controlBoard{1};

    Drivetrain swerve{};
    Elevator elevator{};
    nt::StructPublisher<frc::Pose3d> stage1Publisher = nt::NetworkTableInstance::GetDefault().GetTable("SimRobot")->GetStructTopic<frc::Pose3d>("stage_1").Publish();
    nt::StructPublisher<frc::Pose3d> carriagePublisher = nt::NetworkTableInstance::GetDefault().GetTable("SimRobot")->GetStructTopic<frc::Pose3d>("carriage").Publish();
    Wrist wrist{};
    nt::StructPublisher<frc::Pose3d> clawPublisher = nt::NetworkTableInstance::GetDefault().GetTable("SimRobot")->GetStructTopic<frc::Pose3d>("claw").Publish();
    IO io{};
    Climber climber{};

    frc::SendableChooser<std::string> autoChooser;

    const Position *desiredPosition;

    void AddTeleopButtonControl(const int button, frc2::CommandPtr command)
    {
        controlBoard.Button(button).Debounce(100_ms).WhileTrue(std::move(command).OnlyIf(frc::DriverStation::IsTeleop));
    }
    void AddPositionButtonControl(const Position &position)
    {
        AddTeleopButtonControl
        (
            position.button,
            GoToPositionCommand(position)
        );
    }
};
