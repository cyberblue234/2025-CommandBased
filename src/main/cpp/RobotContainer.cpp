// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include <frc2/command/Commands.h>

RobotContainer::RobotContainer()
{
	ConfigureBindings();
}

void RobotContainer::ConfigureBindings() 
{	
	// swerve.SetDefaultCommand
	// (
	// 	swerve.ApplyRequest([this]() {			
	// 		return std::make_unique<ctre::phoenix6::swerve::requests::FieldCentric>(
	// 			drive.WithVelocityX(units::meters_per_second_t{-gamepad.GetLeftY() * DrivetrainConstants::kMaxSpeed.value()})
	// 			.WithVelocityY(units::meters_per_second_t{-gamepad.GetLeftX() * DrivetrainConstants::kMaxSpeed.value()})
	// 			.WithRotationalRate(units::radians_per_second_t{-gamepad.GetRightX() * DrivetrainConstants::kMaxAngularSpeed.value()}));
	// 	})
	// );

	swerve.SetDefaultCommand(frc2::cmd::Run
	(
		[this]()
		{
			swerve.SetControl(drive.WithVelocityX(units::meters_per_second_t{-gamepad.GetLeftY() * DrivetrainConstants::kMaxSpeed.value()})
				.WithVelocityY(units::meters_per_second_t{-gamepad.GetLeftX() * DrivetrainConstants::kMaxSpeed.value()})
				.WithRotationalRate(units::radians_per_second_t{-gamepad.GetRightX() * DrivetrainConstants::kMaxAngularSpeed.value()}));
		}, {&swerve}
	));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
	return frc2::cmd::Print("No autonomous command configured");
}
