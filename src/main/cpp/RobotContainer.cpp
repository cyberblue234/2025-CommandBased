// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

RobotContainer::RobotContainer()
{
	ConfigureBindings();

	std::vector<std::string> autos = pathplanner::AutoBuilder::getAllAutoNames();
    autoChooser.SetDefaultOption("Nothing", "Nothing");
	for (auto i = autos.begin(); i != autos.end(); ++i)
	{
		autoChooser.AddOption(*i, *i);
	}

	frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);
	frc::SmartDashboard::PutData("Swerve", &swerve);
	frc::SmartDashboard::PutData("Elevator", &elevator);
}

void RobotContainer::ConfigureBindings() 
{	
	swerve.SetDefaultCommand
	(
		swerve.DriveWithSpeedsCommand([this]
		{
			frc::ChassisSpeeds speeds;
			speeds.vx = -gamepad.GetLeftY() * DrivetrainConstants::kMaxSpeed;
			speeds.vy = -gamepad.GetLeftX() * DrivetrainConstants::kMaxSpeed;
			speeds.omega = -gamepad.GetRightX() * DrivetrainConstants::kMaxAngularSpeed;
			return speeds;
		})
	);
	
	elevator.SetDefaultCommand
	(
		elevator.StopMotorsCommand()
	);

	gamepad.A().WhileTrue(elevator.GoToHeightCommand(3_ft));
	controlBoard.AxisGreaterThan(ControlsConstants::kManualElevatorAxis, 0.5).WhileTrue(elevator.SetMotorsCommand(ElevatorConstants::kElevatorPower));
	controlBoard.AxisLessThan(ControlsConstants::kManualElevatorAxis, -0.5).WhileTrue(elevator.SetMotorsCommand(-ElevatorConstants::kElevatorPower));

	// Sys Id triggers. Only works during Test mode.
	gamepad.Back().OnTrue(frc2::cmd::RunOnce(SignalLogger::Start).OnlyIf(frc::DriverStation::IsTest));
	gamepad.Start().OnTrue(frc2::cmd::RunOnce(SignalLogger::Stop).OnlyIf(frc::DriverStation::IsTest));
	gamepad.POVUp().WhileTrue(swerve.SysIdQuasistatic(frc2::sysid::Direction::kForward).OnlyIf(frc::DriverStation::IsTest));
	gamepad.POVRight().WhileTrue(swerve.SysIdQuasistatic(frc2::sysid::Direction::kReverse).OnlyIf(frc::DriverStation::IsTest));
	gamepad.POVDown().WhileTrue(swerve.SysIdDynamic(frc2::sysid::Direction::kForward).OnlyIf(frc::DriverStation::IsTest));
	gamepad.POVLeft().WhileTrue(swerve.SysIdDynamic(frc2::sysid::Direction::kReverse).OnlyIf(frc::DriverStation::IsTest));
}

std::optional<frc2::CommandPtr> RobotContainer::GetAutonomousCommand()
{
	std::string auton = autoChooser.GetSelected();
    if (auton == "Nothing") return {};
    return pathplanner::PathPlannerAuto(auton).ToPtr();
}
