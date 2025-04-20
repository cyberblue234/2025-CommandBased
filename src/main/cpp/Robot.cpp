// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>

Robot::Robot() {}

void Robot::RobotInit()
{
	ctre::phoenix6::SignalLogger::Start();
}

void Robot::RobotPeriodic()
{
	frc2::CommandScheduler::GetInstance().Run();
	frc::SmartDashboard::PutNumber("Voltage", frc::RobotController::GetBatteryVoltage().value());
	container.UpdateSimulatedRobotComponents();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

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
