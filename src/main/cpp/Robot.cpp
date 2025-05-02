// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>

Robot::Robot() {}

void Robot::RobotInit()
{
	ctre::phoenix6::SignalLogger::Start();
	frc::SmartDashboard::PutNumber("testPoint/X", 0);
	frc::SmartDashboard::PutNumber("testPoint/Y", 0);
	frc::SmartDashboard::PutNumber("testPoint/Z", 0);
	frc::SmartDashboard::PutNumber("testPoint/Roll", 0);
	frc::SmartDashboard::PutNumber("testPoint/Pitch", 0);
	frc::SmartDashboard::PutNumber("testPoint/Yaw", 0);
}

void Robot::RobotPeriodic()
{
	frc2::CommandScheduler::GetInstance().Run();
	frc::SmartDashboard::PutNumber("Voltage", frc::RobotController::GetBatteryVoltage().value());
	container.UpdateSimulatedRobotComponents();
	container.UpdateTelemetry();

	testPoint.Set(frc::Pose3d
	(
		frc::SmartDashboard::GetNumber("testPoint/X", 0) * 1_m,
		frc::SmartDashboard::GetNumber("testPoint/Y", 0) * 1_m,
		frc::SmartDashboard::GetNumber("testPoint/Z", 0) * 1_m,
		frc::Rotation3d
		(
			frc::SmartDashboard::GetNumber("testPoint/Roll", 0)  * 1_deg,
			frc::SmartDashboard::GetNumber("testPoint/Pitch", 0) * 1_deg,
			frc::SmartDashboard::GetNumber("testPoint/Yaw", 0)   * 1_deg
		)
	));
}

void Robot::DisabledInit() 
{
	container.SetAutoPathPublisher();
}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() 
{
	container.ClearAutoPathPublisher();
}

void Robot::AutonomousInit()
{
	autonCommand = container.GetAutonomousCommand();

	if (autonCommand)
	{
		autonCommand->Schedule();
	}
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit()
{
	if (autonCommand)
	{
		autonCommand->Cancel();
	}
}

void Robot::TeleopInit()
{
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::TestInit()
{
	frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

void Robot::SimulationPeriodic()
{
	// Drivetrain *swerve = container.GetSwerve();
	// std::vector<units::ampere_t> totalCurrents;
	// for (auto &module : swerve->GetModules())
	// {
	// 	totalCurrents.push_back(module->GetDriveMotor().GetSimState().GetSupplyCurrent());
	// }
	// Elevator *elevator = container.GetElevator();
	// totalCurrents.push_back(elevator->GetMotor1()->GetSimState().GetSupplyCurrent());
	// totalCurrents.push_back(elevator->GetMotor2()->GetSimState().GetSupplyCurrent());
	// frc::sim::RoboRioSim::SetVInVoltage(frc::sim::BatterySim::Calculate(13_V, 0.02_Ohm, totalCurrents));
}

#ifndef RUNNING_FRC_TESTS
int main()
{
	return frc::StartRobot<Robot>();
}
#endif
