#pragma once

#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"

class Coral
{
public:
    const frc::Pose3d &UpdatePhysics(const units::turns_per_second_t &ioMotorSpeed, const frc::Pose2d &robotPose, const units::meter_t &elevatorHeight, const units::degree_t &wristAngle)
    {
        units::meter_t xOffset = units::math::cos(RobotSim::DigitalRobot::kCoralThetaOffset - wristAngle) * RobotSim::DigitalRobot::kCoralRadius + RobotSim::DigitalRobot::kCoralXMidpoint;
        units::meter_t yOffset = units::math::sin(RobotSim::DigitalRobot::kCoralThetaOffset - wristAngle) * RobotSim::DigitalRobot::kCoralRadius + RobotSim::DigitalRobot::kCoralYMidpoint;
        units::meter_t xPose = robotPose.X() + units::math::cos(robotPose.Rotation().Degrees()) * xOffset;
        units::meter_t yPose = robotPose.Y() + units::math::sin(robotPose.Rotation().Degrees()) * xOffset;
        units::meter_t zPose = elevatorHeight + yOffset;
        if (isInClaw)
        {
            const std::vector<units::meters_per_second_t> speeds = DecomposeSpeed(ioMotorSpeed * (1_in / 1_tr) / ClawConstants::kIOGearRatio, wristAngle, robotPose.Rotation().Degrees());
            xSpeed = speeds[0];
            ySpeed = speeds[1];
            zSpeed = speeds[2];
            frc::SmartDashboard::PutNumber("coralsim/xSpeed", xSpeed.value());
            frc::SmartDashboard::PutNumber("coralsim/ySpeed", ySpeed.value());
            frc::SmartDashboard::PutNumber("coralsim/zSpeed", zSpeed.value());
        }
        
        units::meter_t xTranslation = xSpeed * 0.02_s;
        units::meter_t yTranslation = ySpeed * 0.02_s;
        units::meter_t zTranslation = zSpeed * 0.02_s;
        if (!isInClaw && !isDead) zSpeed -= units::standard_gravity_t{1} * 0.02_s;

        frc::Rotation3d rotation;
        if (isInClaw)
        {
            rotation = {0_deg, wristAngle + RobotSim::DigitalRobot::kCoralVisualOffset, robotPose.Rotation().Degrees()};
            pose = {(xPose - oldXPose) + pose.X() + xTranslation, (yPose - oldYPose) + pose.Y() + yTranslation, (zPose - oldZPose) + pose.Z() + zTranslation, rotation};
        }
        else
        {
            rotation = pose.Rotation();
            pose = {pose.X() + xTranslation, pose.Y() + yTranslation, pose.Z() + zTranslation, rotation};
        }
        
        oldXPose = xPose;
        oldYPose = yPose;
        oldZPose = zPose;
       
        units::meter_t distance = units::math::sqrt(units::math::pow<2>(pose.X() - xPose) + units::math::pow<2>(pose.Y() - yPose) + units::math::pow<2>(pose.Z() - zPose));
        if (distance > 9.5_in) isInClaw = false;

        if (pose.Z() <= 0_m)
        {
            xSpeed = 0_mps;
            ySpeed = 0_mps;
            zSpeed = 0_mps;
            isDead = true;
        } 
        
        return pose;
    }

    const std::vector<units::meters_per_second_t> DecomposeSpeed(const units::meters_per_second_t &speed, const units::degree_t &wristAngle, const units::degree_t &robotAngle)
    {
        units::dimensionless_t translationComponent = -units::math::cos(wristAngle + RobotSim::DigitalRobot::kCoralVisualOffset);
        units::meters_per_second_t xSpeed = translationComponent * units::math::cos(robotAngle) * speed;
        units::meters_per_second_t ySpeed = translationComponent * units::math::sin(robotAngle) * speed;
        units::meters_per_second_t zSpeed = units::math::sin(wristAngle + RobotSim::DigitalRobot::kCoralVisualOffset) * speed;
        return {xSpeed, ySpeed, zSpeed};
    }

    bool IsInClaw() { return isInClaw; }

private:
    frc::Pose3d pose;
    bool isInClaw = true;
    bool isDead = false;

    units::meters_per_second_t xSpeed;
    units::meters_per_second_t ySpeed;
    units::meters_per_second_t zSpeed;
    units::meter_t oldXPose;
    units::meter_t oldYPose;
    units::meter_t oldZPose;
};

class CoralManager
{
public:
    void InstantiateCoral()
    {
        coralHolder.push_back(Coral{});
    }

    void UpdateCoral(const units::turns_per_second_t &ioMotorSpeed, const frc::Pose2d &robotPose, const units::meter_t &elevatorHeight, const units::degree_t &wristAngle)
    {
        std::vector<frc::Pose3d> poses;
        for (auto i = coralHolder.begin(); i != coralHolder.end();)
        {
            Coral &coral = *i;
            const frc::Pose3d &pose = coral.UpdatePhysics(ioMotorSpeed, robotPose, elevatorHeight, wristAngle);
            if (pose == frc::Pose3d{})
            {
                i = coralHolder.erase(i);
                continue;
            }
            poses.push_back(pose);
            ++i;
        }
        coralPublisher.Set(poses);
    }

    const bool AreAnyCoralInClaw()
    {
        for (Coral coral : coralHolder)
        {
            if (coral.IsInClaw()) return true;
        }
        return false;
    }

private:
    std::vector<Coral> coralHolder;

    nt::StructArrayPublisher<frc::Pose3d> coralPublisher = nt::NetworkTableInstance::GetDefault().GetTable("SimRobot")->GetStructArrayTopic<frc::Pose3d>("coralHolder").Publish();
};