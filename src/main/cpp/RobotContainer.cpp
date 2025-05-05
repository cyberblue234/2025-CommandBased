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

	autoChooser.OnChange
	(
		[this] (std::string auton)
		{
			SetAutoPathPublisher(auton);
		}
	);

	NamedCommands::registerCommand("L1", GoToPositionCommand(Positions::L1));
	NamedCommands::registerCommand("L2", GoToPositionCommand(Positions::L2));
	NamedCommands::registerCommand("L3", GoToPositionCommand(Positions::L3));
	NamedCommands::registerCommand("L4", GoToPositionCommand(Positions::L4));
	NamedCommands::registerCommand("AlgaeLow", GoToPositionCommand(Positions::AlgaeLow));
	NamedCommands::registerCommand("AlgaeHigh", GoToPositionCommand(Positions::AlgaeHigh));
	NamedCommands::registerCommand("CoralStation", GoToPositionCommand(Positions::CoralStation));
	NamedCommands::registerCommand("Processor", GoToPositionCommand(Positions::Processor));
	NamedCommands::registerCommand("Barge", GoToPositionCommand(Positions::Barge));
	NamedCommands::registerCommand("CoralHome", GoToPositionCommand(Positions::CoralHome));
	NamedCommands::registerCommand("AlgaeHome", GoToPositionCommand(Positions::AlgaeHome));
	NamedCommands::registerCommand("IO", io.IOAtPosition([this] { return desiredPosition; }).OnlyWhile([this] { return elevator.IsAtPosition() && wrist.IsAtPosition();}));

	frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);
	frc::SmartDashboard::PutData("Swerve", &swerve);
	frc::SmartDashboard::PutData("Elevator", &elevator);
	frc::SmartDashboard::PutData("Wrist", &wrist);
	frc::SmartDashboard::PutData("IO", &io);

	addCoral.Debounce(100_ms).OnTrue(frc2::cmd::RunOnce([this] 
	{
		if (coralManager.AreAnyCoralInClaw() == false) 
		{
			coralManager.InstantiateCoral();
		} 
	}).IgnoringDisable(true));
	addCoral.Debounce(100_ms).OnFalse(frc2::cmd::RunOnce([this] 
	{
		if (coralManager.AreAnyCoralInClaw() == true)
		{
			coralManager.DeleteCoralInClaw();
		}
	}).IgnoringDisable(true));
	
	simChangeCoralState.OnTrue(frc2::cmd::RunOnce([this] 
	{
		io.SetSimProximitySensorDistance(0_m);
	}).IgnoringDisable(true).OnlyIf(frc::RobotBase::IsSimulation));
	simChangeCoralState.OnFalse(frc2::cmd::RunOnce([this] 
	{
		io.SetSimProximitySensorDistance(kProximityThreshold + 1_in);
	}).IgnoringDisable(true).OnlyIf(frc::RobotBase::IsSimulation));
}

void RobotContainer::ConfigureBindings() 
{	
	swerve.SetDefaultCommand
	(
		swerve.DriveWithSpeedsCommand([this]
		{
			frc::ChassisSpeeds speeds;
			double speedAdjust = gamepad.GetRightTriggerAxis() > 0.5 ? 0.4 : 1;
			speeds.vx = -gamepad.GetLeftY() * DrivetrainConstants::kMaxSpeed * speedAdjust;
			speeds.vy = -gamepad.GetLeftX() * DrivetrainConstants::kMaxSpeed * speedAdjust;
			speeds.omega = -gamepad.GetRightX() * DrivetrainConstants::kMaxAngularSpeed * speedAdjust;
			return speeds;
		}, frc::RobotBase::IsReal(), gamepad.GetRightTriggerAxis() > 0.5)
	);
	
	elevator.SetDefaultCommand
	(
		elevator.StopMotorsCommand()
	);

	wrist.SetDefaultCommand
	(
		wrist.StopWristMotorCommand()
	);

	io.SetDefaultCommand
	(
		io.StopIOMotorCommand()
	);
	
	climber.SetDefaultCommand
	(
		climber.StopMotorCommand()
	);

	AddPositionButtonControl(Positions::L1);
	AddPositionButtonControl(Positions::L2);
	AddPositionButtonControl(Positions::L3);
	AddPositionButtonControl(Positions::L4);
	AddPositionButtonControl(Positions::AlgaeLow);
	AddPositionButtonControl(Positions::AlgaeHigh);
	AddPositionButtonControl(Positions::CoralStation);
	AddPositionButtonControl(Positions::Processor);
	AddPositionButtonControl(Positions::Barge);
	AddPositionButtonControl(Positions::CoralHome);
	AddPositionButtonControl(Positions::AlgaeHome);

	AddTeleopButtonControl(ControlsConstants::kIOButton, io.IOAtPosition([this] { return desiredPosition; }).OnlyWhile([this] { return elevator.IsAtPosition() && wrist.IsAtPosition(); }).OnlyIf(frc::DriverStation::IsTeleopEnabled));

	gamepad.Y().Debounce(100_ms).OnTrue(frc2::cmd::RunOnce([this] { swerve.ResetRotation(0_deg); }));

	// Manual elevator controls
	controlBoard.AxisGreaterThan(ControlsConstants::kManualElevatorAxis, 0.5).WhileTrue(elevator.SetMotorsCommand(ElevatorConstants::kElevatorPower).OnlyIf(frc::DriverStation::IsTeleopEnabled));
	controlBoard.AxisLessThan(ControlsConstants::kManualElevatorAxis, -0.5).WhileTrue(elevator.SetMotorsCommand(-ElevatorConstants::kElevatorPower).OnlyIf(frc::DriverStation::IsTeleopEnabled));
	// Manual wrist controls
	controlBoard.AxisGreaterThan(ControlsConstants::kManualWristAxis, 0.5).WhileTrue(wrist.SetWristPowerCommand(ClawConstants::kWristPower).OnlyIf(frc::DriverStation::IsTeleopEnabled));
	controlBoard.AxisLessThan(ControlsConstants::kManualWristAxis, -0.5).WhileTrue(wrist.SetWristPowerCommand(-ClawConstants::kWristPower).OnlyIf(frc::DriverStation::IsTeleopEnabled));
	// Manual IO controls
	controlBoard.AxisGreaterThan(ControlsConstants::kManualIntakeAxis, 0.5).WhileTrue(io.SetIOPowerCommand(ClawConstants::kManualIOPower).OnlyIf(frc::DriverStation::IsTeleopEnabled));
	controlBoard.AxisLessThan(ControlsConstants::kManualIntakeAxis, -0.5).WhileTrue(io.SetIOPowerCommand(-ClawConstants::kManualIOPower).OnlyIf(frc::DriverStation::IsTeleopEnabled));
	// Climber controls
	controlBoard.AxisGreaterThan(ControlsConstants::kClimberAxis, 0.5).WhileTrue(climber.SetPowerCommand(ClimberConstants::kClimberPower).OnlyIf(frc::DriverStation::IsTeleopEnabled));
	controlBoard.AxisLessThan(ControlsConstants::kClimberAxis, -0.5).WhileTrue(climber.SetPowerCommand(-ClimberConstants::kClimberPower).OnlyIf(frc::DriverStation::IsTeleopEnabled));

	// Sys Id triggers. Only works during Test mode.
	gamepad.Back().Debounce(100_ms).OnTrue(frc2::cmd::RunOnce(SignalLogger::Start).OnlyIf(frc::DriverStation::IsTest));
	gamepad.Start().Debounce(100_ms).OnTrue(frc2::cmd::RunOnce(SignalLogger::Stop).OnlyIf(frc::DriverStation::IsTest));
	gamepad.POVUp().Debounce(100_ms).WhileTrue(swerve.SysIdQuasistatic(frc2::sysid::Direction::kForward).OnlyIf(frc::DriverStation::IsTestEnabled));
	gamepad.POVRight().Debounce(100_ms).WhileTrue(swerve.SysIdQuasistatic(frc2::sysid::Direction::kReverse).OnlyIf(frc::DriverStation::IsTestEnabled));
	gamepad.POVDown().Debounce(100_ms).WhileTrue(swerve.SysIdDynamic(frc2::sysid::Direction::kForward).OnlyIf(frc::DriverStation::IsTestEnabled));
	gamepad.POVLeft().Debounce(100_ms).WhileTrue(swerve.SysIdDynamic(frc2::sysid::Direction::kReverse).OnlyIf(frc::DriverStation::IsTestEnabled));
	gamepad.X().Debounce(100_ms).OnTrue(frc2::cmd::RunOnce([this] { swerve.SetSysIdRoutineToApply(swerve.GetTranslationRoutine(), "translation"); }).OnlyIf(frc::DriverStation::IsTest));
	gamepad.A().Debounce(100_ms).OnTrue(frc2::cmd::RunOnce([this] { swerve.SetSysIdRoutineToApply(swerve.GetSteerRoutine(), "steer"); }).OnlyIf(frc::DriverStation::IsTest));
	gamepad.B().Debounce(100_ms).OnTrue(frc2::cmd::RunOnce([this] { swerve.SetSysIdRoutineToApply(swerve.GetRotationRoutine(), "rotation"); }).OnlyIf(frc::DriverStation::IsTest));
}

std::optional<frc2::CommandPtr> RobotContainer::GetAutonomousCommand()
{
	std::string auton = GetAutoPathName();
    if (auton == "Nothing") return {};
    return pathplanner::PathPlannerAuto(auton).ToPtr();
}
