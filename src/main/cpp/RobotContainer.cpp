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
}

void RobotContainer::ConfigureBindings() 
{	
	swerve.SetDefaultCommand
	(
		swerve.ApplyRequest([this]() {			
			return std::make_shared<ctre::phoenix6::swerve::requests::FieldCentric>(
				drive.WithVelocityX(-gamepad.GetLeftY() * DrivetrainConstants::kMaxSpeed)
				.WithVelocityY(-gamepad.GetLeftX() * DrivetrainConstants::kMaxSpeed)
				.WithRotationalRate(-gamepad.GetRightX() * DrivetrainConstants::kMaxAngularSpeed));
		})
	);
}

std::optional<frc2::CommandPtr> RobotContainer::GetAutonomousCommand()
{
	std::string auton = autoChooser.GetSelected();
    if (auton == "Nothing") return {};
    return pathplanner::PathPlannerAuto(auton).ToPtr();
}
