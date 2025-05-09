// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <optional>

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>

#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>

#include "RobotContainer.h"

class Robot : public frc::TimedRobot
{
public:
    Robot();
    void RobotInit() override;
    void RobotPeriodic() override;
    void DisabledInit() override;
    void DisabledPeriodic() override;
    void DisabledExit() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void AutonomousExit() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void TeleopExit() override;
    void TestInit() override;
    void TestPeriodic() override;
    void TestExit() override;
    void SimulationPeriodic() override;

private:
    std::optional<frc2::CommandPtr> autonCommand;

    nt::StructPublisher<frc::Pose3d> testPoint = nt::NetworkTableInstance::GetDefault().GetTable("SimRobot")->GetStructTopic<frc::Pose3d>("testPoint").Publish();

    RobotContainer container;
};
