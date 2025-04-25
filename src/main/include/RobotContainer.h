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
                this->desiredPosition = desiredPosition;
            }
        );
    }

    void UpdateSimulatedRobotComponents()
    {
        stage1Publisher.Set(frc::Pose3d{0_m, 0_m, (elevator.GetHeight() - ElevatorConstants::kHeightOffset) / 2, frc::Rotation3d{0_deg, 0_deg, 0_deg}});
        carriagePublisher.Set(frc::Pose3d{0_m, 0_m, elevator.GetHeight() - ElevatorConstants::kHeightOffset, frc::Rotation3d{0_deg, 0_deg, 0_deg}});
        clawPublisher.Set(frc::Pose3d{RobotSim::DigitalRobot::kClawX, 0_m, elevator.GetHeight() - ElevatorConstants::kHeightOffset + RobotSim::DigitalRobot::kClawY, frc::Rotation3d{0_deg, wrist.GetCurrentAngle(), 0_deg}});
        stage1DesiredPublisher.Set(frc::Pose3d{0_m, 0_m, elevator.GetHeightSetpoint() / 2, frc::Rotation3d{0_deg, 0_deg, 0_deg}});
        carriageDesiredPublisher.Set(frc::Pose3d{0_m, 0_m, elevator.GetHeightSetpoint(), frc::Rotation3d{0_deg, 0_deg, 0_deg}});
        clawDesiredPublisher.Set(frc::Pose3d{RobotSim::DigitalRobot::kClawX, 0_m, elevator.GetHeightSetpoint() + RobotSim::DigitalRobot::kClawY, frc::Rotation3d{0_deg, wrist.GetAngleSetpoint(), 0_deg}});
        if (io.IsCoralInClaw())
        {
            frc::Pose2d robotPose = swerve.GetState().Pose;
            units::meter_t xOffset = units::math::cos(RobotSim::DigitalRobot::kCoralThetaOffset - wrist.GetCurrentAngle()) * RobotSim::DigitalRobot::kCoralRadius + RobotSim::DigitalRobot::kCoralXMidpoint;
            coralPublisher.Set(frc::Pose3d{robotPose.X() + xOffset * units::math::cos(robotPose.Rotation().Degrees()), 
                robotPose.Y() + units::math::sin(robotPose.Rotation().Degrees()) * xOffset , elevator.GetHeight() + units::math::sin(RobotSim::DigitalRobot::kCoralThetaOffset- wrist.GetCurrentAngle()) * RobotSim::DigitalRobot::kCoralRadius + RobotSim::DigitalRobot::kCoralYMidpoint, 
                frc::Rotation3d{0_deg, wrist.GetCurrentAngle() + 20_deg, robotPose.Rotation().Degrees()}});
        }
        else
        {
            coralPublisher.Set(frc::Pose3d{});
        }
    }

    void UpdateTelemetry()
    {
        frc::SmartDashboard::PutString("Robot/desiredPosition", desiredPosition.to_string());
    }

    std::string GetAutoPathName()
    {
        return autoChooser.GetSelected();
    }

    void SetAutoPathPublisher(std::string auton="")
    {
        if (frc::DriverStation::IsDisabled())
        {
            if (auton == "")
            {
                auton = GetAutoPathName();
            }

            if (auton == "Nothing")
            {
                ClearAutoPathPublisher();
                return;
            }
            std::vector<std::shared_ptr<pathplanner::PathPlannerPath>> paths = pathplanner::PathPlannerAuto::getPathGroupFromAutoFile(auton);
            
            std::vector<frc::Pose2d> allPathsPoses{};
            for (std::shared_ptr<pathplanner::PathPlannerPath> path : paths)
            {
                for (frc::Pose2d pose : path->getPathPoses())
                {
                    allPathsPoses.push_back(FlipPose(pose));
                }
            }
            autoPathPublisher.Set(allPathsPoses);
        }
    }

    void ClearAutoPathPublisher()
    {
        autoPathPublisher.Set(std::vector<frc::Pose2d>{});
    }

private:
    void ConfigureBindings();

    frc2::CommandXboxController gamepad{0};
    frc2::CommandJoystick controlBoard{1};

    Drivetrain swerve{};
    Elevator elevator{};
    nt::StructPublisher<frc::Pose3d> stage1Publisher = nt::NetworkTableInstance::GetDefault().GetTable("SimRobot")->GetStructTopic<frc::Pose3d>("stage_1").Publish();
    nt::StructPublisher<frc::Pose3d> carriagePublisher = nt::NetworkTableInstance::GetDefault().GetTable("SimRobot")->GetStructTopic<frc::Pose3d>("carriage").Publish();
    nt::StructPublisher<frc::Pose3d> stage1DesiredPublisher = nt::NetworkTableInstance::GetDefault().GetTable("SimRobot")->GetStructTopic<frc::Pose3d>("stage_1-desired").Publish();
    nt::StructPublisher<frc::Pose3d> carriageDesiredPublisher = nt::NetworkTableInstance::GetDefault().GetTable("SimRobot")->GetStructTopic<frc::Pose3d>("carriage-desired").Publish();
    Wrist wrist{};
    nt::StructPublisher<frc::Pose3d> clawPublisher = nt::NetworkTableInstance::GetDefault().GetTable("SimRobot")->GetStructTopic<frc::Pose3d>("claw").Publish();
    nt::StructPublisher<frc::Pose3d> clawDesiredPublisher = nt::NetworkTableInstance::GetDefault().GetTable("SimRobot")->GetStructTopic<frc::Pose3d>("claw-desired").Publish();
    nt::StructPublisher<frc::Pose3d> coralPublisher = nt::NetworkTableInstance::GetDefault().GetTable("SimRobot")->GetStructTopic<frc::Pose3d>("coral").Publish();
    IO io{};
    Climber climber{};

    frc::SendableChooser<std::string> autoChooser;
    nt::StructArrayPublisher<frc::Pose2d> autoPathPublisher = nt::NetworkTableInstance::GetDefault().GetTable("Auto")->GetStructArrayTopic<frc::Pose2d>("autoPath").Publish();

    Position desiredPosition;

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
