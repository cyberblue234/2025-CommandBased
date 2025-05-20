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

	swerve.limelightPoseEstimatesSupplier = [this] ()
	{
		ctre::phoenix6::hardware::Pigeon2 &pigeon = swerve.GetPigeon2();
		units::degree_t yaw = pigeon.GetYaw().GetValue();
		units::degree_t pitch = pigeon.GetPitch().GetValue();
		units::degree_t roll = pigeon.GetRoll().GetValue();
		units::degrees_per_second_t yawRate = pigeon.GetAngularVelocityZWorld().GetValue();

		limelightLow.SetRobotOrientation(yaw, yawRate, pitch, 0_deg_per_s, roll, 0_deg_per_s);
		limelightHigh.SetRobotOrientation(yaw, yawRate, pitch, 0_deg_per_s, roll, 0_deg_per_s);
		return std::vector<PoseEstimate>{ limelightLow.GetPose(), limelightHigh.GetPose()};
	};

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
			speeds.vx = -gamepad.GetLeftY() * DrivetrainConstants::kMaxSpeed;
			speeds.vy = -gamepad.GetLeftX() * DrivetrainConstants::kMaxSpeed;
			speeds.omega = -gamepad.GetRightX() * DrivetrainConstants::kMaxAngularSpeed;
			return speeds;
		}, true)
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

	AddControlBoardPositionCommand(Positions::L1);
	AddControlBoardPositionCommand(Positions::L2);
	AddControlBoardPositionCommand(Positions::L3);
	AddControlBoardPositionCommand(Positions::L4);
	AddControlBoardPositionCommand(Positions::AlgaeLow);
	AddControlBoardPositionCommand(Positions::AlgaeHigh);
	AddControlBoardPositionCommand(Positions::CoralStation);
	AddControlBoardPositionCommand(Positions::Processor);
	AddControlBoardPositionCommand(Positions::CoralHome);
	AddControlBoardPositionCommand(Positions::AlgaeHome);

	AddControlBoardCommand(Positions::Barge.button, 
		frc2::cmd::Parallel
		(
			elevator.GoToPositionCommand(Positions::Barge),
			wrist.GoToPositionCommand(Positions::Barge).OnlyIf([this] { return elevator.GetHeight() > 2.5_ft; }),
			io.SetIOPowerCommand(IOConstants::kManualIOPower).OnlyWhile([this] { return elevator.GetHeight() > 3.5_ft; })
		).BeforeStarting
		(
			[this]
			{
				this->desiredPosition = Positions::Barge;
			}
		)
	);

	AddControlBoardCommand(ControlsConstants::kIOButton, io.IOAtPosition([this] { return desiredPosition; }).OnlyWhile([this] { return elevator.IsAtPosition() && wrist.IsAtPosition(); }).OnlyIf(frc::DriverStation::IsTeleopEnabled));

	gamepad.POVUp().Debounce(40_ms).WhileTrue(swerve.DriveWithSpeedsCommand([this] { frc::ChassisSpeeds speeds; speeds.vx = 1_mps; return speeds; }, false).OnlyIf(frc::DriverStation::IsTeleopEnabled).WithName("Test"));
	gamepad.POVRight().Debounce(40_ms).WhileTrue(swerve.DriveWithSpeedsCommand([this] { frc::ChassisSpeeds speeds; speeds.vy = -1_mps; return speeds; }, false).OnlyIf(frc::DriverStation::IsTeleopEnabled));
	gamepad.POVDown().Debounce(40_ms).WhileTrue(swerve.DriveWithSpeedsCommand([this] { frc::ChassisSpeeds speeds; speeds.vx = -1_mps; return speeds; }, false).OnlyIf(frc::DriverStation::IsTeleopEnabled));
	gamepad.POVLeft().Debounce(40_ms).WhileTrue(swerve.DriveWithSpeedsCommand([this] { frc::ChassisSpeeds speeds; speeds.vy = 1_mps; return speeds; }, false).OnlyIf(frc::DriverStation::IsTeleopEnabled));

	gamepad.Y().Debounce(40_ms).OnTrue(frc2::cmd::RunOnce([this] { swerve.ResetRotation(0_deg); }));

	// Manual elevator controls
	controlBoard.AxisGreaterThan(ControlsConstants::kManualElevatorAxis, 0.5).WhileTrue(elevator.SetMotorsCommand(ElevatorConstants::kElevatorPower).OnlyIf(frc::DriverStation::IsTeleopEnabled));
	controlBoard.AxisLessThan(ControlsConstants::kManualElevatorAxis, -0.5).WhileTrue(elevator.SetMotorsCommand(-ElevatorConstants::kElevatorPower).OnlyIf(frc::DriverStation::IsTeleopEnabled));
	// Manual wrist controls
	controlBoard.AxisGreaterThan(ControlsConstants::kManualWristAxis, 0.5).WhileTrue(wrist.SetWristPowerCommand(WristConstants::kWristPower).OnlyIf(frc::DriverStation::IsTeleopEnabled));
	controlBoard.AxisLessThan(ControlsConstants::kManualWristAxis, -0.5).WhileTrue(wrist.SetWristPowerCommand(-WristConstants::kWristPower).OnlyIf(frc::DriverStation::IsTeleopEnabled));
	// Manual IO controls
	controlBoard.AxisGreaterThan(ControlsConstants::kManualIntakeAxis, 0.5).WhileTrue(io.SetIOPowerCommand(IOConstants::kManualIOPower).OnlyIf(frc::DriverStation::IsTeleopEnabled));
	controlBoard.AxisLessThan(ControlsConstants::kManualIntakeAxis, -0.5).WhileTrue(io.SetIOPowerCommand(-IOConstants::kManualIOPower).OnlyIf(frc::DriverStation::IsTeleopEnabled));
	// Climber controls
	controlBoard.AxisGreaterThan(ControlsConstants::kClimberAxis, 0.5).WhileTrue(climber.SetPowerCommand(ClimberConstants::kClimberPower).OnlyIf(frc::DriverStation::IsTeleopEnabled));
	controlBoard.AxisLessThan(ControlsConstants::kClimberAxis, -0.5).WhileTrue(climber.SetPowerCommand(-ClimberConstants::kClimberPower).OnlyIf(frc::DriverStation::IsTeleopEnabled));

	// Sys Id triggers. Only works during Test mode.
	gamepad.Back().Debounce(40_ms).OnTrue(frc2::cmd::RunOnce(SignalLogger::Start).OnlyIf(frc::DriverStation::IsTest));
	gamepad.Start().Debounce(40_ms).OnTrue(frc2::cmd::RunOnce(SignalLogger::Stop).OnlyIf(frc::DriverStation::IsTest));
	gamepad.POVUp().Debounce(40_ms).WhileTrue(swerve.SysIdQuasistatic(frc2::sysid::Direction::kForward).OnlyIf(frc::DriverStation::IsTestEnabled).WithName("Quasistatic Forward"));
	gamepad.POVRight().Debounce(40_ms).WhileTrue(swerve.SysIdQuasistatic(frc2::sysid::Direction::kReverse).OnlyIf(frc::DriverStation::IsTestEnabled).WithName("Quasistatic Reverse"));
	gamepad.POVDown().Debounce(40_ms).WhileTrue(swerve.SysIdDynamic(frc2::sysid::Direction::kForward).OnlyIf(frc::DriverStation::IsTestEnabled).WithName("Dynamic Forward"));
	gamepad.POVLeft().Debounce(40_ms).WhileTrue(swerve.SysIdDynamic(frc2::sysid::Direction::kReverse).OnlyIf(frc::DriverStation::IsTestEnabled).WithName("Dynamic Reverse"));
	gamepad.X().Debounce(40_ms).OnTrue(frc2::cmd::RunOnce([this] { swerve.SetSysIdRoutineToApply(Drivetrain::SysIdRoutines::Translation); }).OnlyIf(frc::DriverStation::IsTest));
	gamepad.A().Debounce(40_ms).OnTrue(frc2::cmd::RunOnce([this] { swerve.SetSysIdRoutineToApply(Drivetrain::SysIdRoutines::Rotation); }).OnlyIf(frc::DriverStation::IsTest));
	gamepad.B().Debounce(40_ms).OnTrue(frc2::cmd::RunOnce([this] { swerve.SetSysIdRoutineToApply(Drivetrain::SysIdRoutines::Steer); }).OnlyIf(frc::DriverStation::IsTest));
}

std::optional<frc2::CommandPtr> RobotContainer::GetAutonomousCommand()
{
	std::string auton = GetAutoPathName();
    if (auton == "Nothing") return {};
    return pathplanner::PathPlannerAuto(auton).ToPtr();
}
