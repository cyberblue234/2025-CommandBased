#pragma once

#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"


enum States
{
    InClaw, FreeFall, L4, L3, L2, L1, Dead
};

const std::vector<frc::Pose2d> reefPoses
{
    frc::Pose2d{14.40_m, 3.87_m, frc::Rotation2d(180_deg)},
    frc::Pose2d{14.40_m, 4.16_m, frc::Rotation2d(180_deg)},
    frc::Pose2d{11.74_m, 4.16_m, frc::Rotation2d(0_deg)},
    frc::Pose2d{11.74_m, 3.87_m, frc::Rotation2d(0_deg)},
    frc::Pose2d{13.85_m, 5.05_m, frc::Rotation2d(-120_deg)},
    frc::Pose2d{13.59_m, 5.22_m, frc::Rotation2d(-120_deg)},
    frc::Pose2d{12.58_m, 5.22_m, frc::Rotation2d(-60_deg)},
    frc::Pose2d{12.28_m, 5.06_m, frc::Rotation2d(-60_deg)},
    frc::Pose2d{12.28_m, 2.97_m, frc::Rotation2d(60_deg)},
    frc::Pose2d{12.58_m, 2.84_m, frc::Rotation2d(60_deg)},
    frc::Pose2d{13.58_m, 2.84_m, frc::Rotation2d(120_deg)},
    frc::Pose2d{13.85_m, 2.97_m, frc::Rotation2d(120_deg)} 
};

constexpr units::meter_t l4Height = 1.72_m;
constexpr units::meter_t l3Height = 1.15_m;
constexpr units::meter_t l2Height = 0.75_m;
constexpr units::meter_t coralLength = 11.875_in;

class Coral
{
public:
    const frc::Pose3d &UpdatePhysics(const units::turns_per_second_t &ioMotorSpeed, const frc::Pose2d &robotPose, const units::meter_t &elevatorHeight, const units::degree_t &wristAngle)
    {
        units::second_t currentTime = ctre::phoenix6::utils::GetCurrentTime();
        if (lastTime == -1_s)
        {
            lastTime = currentTime - 20_ms;
        }
        units::second_t deltaTime = currentTime - lastTime;

        if (state == FreeFall)
        {
            bool flip = robotPose.X() < pathplanner::FlippingUtil::fieldSizeX / 2;
            frc::Pose2d nearestPose = pose.ToPose2d().Nearest(flip ? FlipFieldPoses(reefPoses) : reefPoses);
            units::meter_t distanceToNearestPose = units::math::sqrt(units::math::pow<2>(pose.X() - nearestPose.X()) + units::math::pow<2>(pose.Y() - nearestPose.Y()));
            if (distanceToNearestPose < 2_in)
            {
                std::cout << "Attached to reef" << std::endl;
                units::meter_t dL4 = units::math::abs(pose.Z() - l4Height);
                units::meter_t dL3 = units::math::abs(pose.Z() - l3Height);
                units::meter_t dL2 = units::math::abs(pose.Z() - l2Height);
                units::meter_t dist = units::math::min(units::math::min(dL4, dL3), dL2);
                if (dist < 1_in)
                {
                    units::meter_t newPoseZ = 0_m;
                    units::degree_t newPosePitch = 0_deg;
                    if (dL4 == dist)
                    {
                        state = L4;
                        newPoseZ = l4Height;
                        newPosePitch = 90_deg;
                        std::cout << "l4" << std::endl;
                    }
                    else if (dL3 == dist)
                    {
                        state = L3;
                        newPoseZ = l3Height;
                        newPosePitch = 35_deg;
                        std::cout << "l3" << std::endl;
                    }
                    else
                    {
                        state = L2;
                        newPoseZ = l2Height;
                        newPosePitch = 35_deg;
                        std::cout << "l2" << std::endl;
                    }
                    pose = frc::Pose3d{nearestPose.X(), nearestPose.Y(), newPoseZ, frc::Rotation3d{0_deg, newPosePitch, nearestPose.Rotation().Degrees()}};
                    xSpeed = 0_mps;
                    ySpeed = 0_mps;
                    zSpeed = 0_mps;
                }
            }
        }

        units::meter_t xOffset;
        units::meter_t yOffset;
        units::meter_t xPose;
        units::meter_t yPose;
        units::meter_t zPose;

        if (state == InClaw)
        {
            xOffset = units::math::cos(RobotSim::DigitalRobot::kCoralThetaOffset - wristAngle) * RobotSim::DigitalRobot::kCoralRadius + RobotSim::DigitalRobot::kCoralXMidpoint;
            yOffset = units::math::sin(RobotSim::DigitalRobot::kCoralThetaOffset - wristAngle) * RobotSim::DigitalRobot::kCoralRadius + RobotSim::DigitalRobot::kCoralYMidpoint;
            xPose = robotPose.X() + units::math::cos(robotPose.Rotation().Degrees()) * xOffset;
            yPose = robotPose.Y() + units::math::sin(robotPose.Rotation().Degrees()) * xOffset;
            zPose = elevatorHeight + yOffset;

            const std::vector<units::meters_per_second_t> speeds = DecomposeSpeed(ioMotorSpeed * (1_in / 1_tr) / ClawConstants::kIOGearRatio, wristAngle, robotPose.Rotation().Degrees());
            xSpeed = speeds[0];
            ySpeed = speeds[1];
            zSpeed = speeds[2];
        }
        
        if (state == InClaw || state == FreeFall)
        {
            units::meter_t xTranslation = xSpeed * deltaTime;
            units::meter_t yTranslation = ySpeed * deltaTime;
            units::meter_t zTranslation = zSpeed * deltaTime;
            if (state == FreeFall) zSpeed -= units::standard_gravity_t(1) * deltaTime;

            frc::Rotation3d rotation;
            if (state == InClaw)
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
        }

        if (state == InClaw)
        {
            units::meter_t distanceFromClaw = units::math::sqrt(units::math::pow<2>(pose.X() - xPose) + units::math::pow<2>(pose.Y() - yPose) + units::math::pow<2>(pose.Z() - zPose));
            if (distanceFromClaw > 9.5_in) 
                state = FreeFall;
        }

        if (state == FreeFall && pose.Z() <= 0_m)
        {
            xSpeed = 0_mps;
            ySpeed = 0_mps;
            zSpeed = 0_mps;
            state = Dead;
        }

        lastTime = currentTime;

        units::meter_t endX = pose.X() + units::math::cos(pose.Rotation().Y()) * units::math::cos(pose.Rotation().Z()) * (coralLength / 2);
        units::meter_t endY = pose.Y() + units::math::cos(pose.Rotation().Y()) * units::math::sin(pose.Rotation().Z()) * (coralLength / 2);
        units::meter_t endZ = pose.Z() - units::math::sin(pose.Rotation().Y()) * (coralLength / 2);
        endPose = frc::Pose3d{endX, endY, endZ, pose.Rotation()};
        
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

    const std::vector<frc::Pose2d> FlipFieldPoses(const std::vector<frc::Pose2d> &poses)
    {
        std::vector<frc::Pose2d> newPoses;
        for (const frc::Pose2d &pose : poses)
        {
            newPoses.push_back(pathplanner::FlippingUtil::flipFieldPose(pose));
        }
        return newPoses;
    }

    States GetState() { return state; }

    const frc::Pose3d &GetPose() { return pose; }
    const frc::Pose3d &GetEndPose() { return endPose; }

private:
    frc::Pose3d pose;
    frc::Pose3d endPose;
    States state = InClaw;

    units::meters_per_second_t xSpeed;
    units::meters_per_second_t ySpeed;
    units::meters_per_second_t zSpeed;
    units::meter_t oldXPose;
    units::meter_t oldYPose;
    units::meter_t oldZPose;

    units::second_t lastTime = -1_s;
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
        std::vector<frc::Pose3d> endPoses;
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
            endPoses.push_back(coral.GetEndPose());
            ++i;
        }
        coralPublisher.Set(poses);
        coralEndsPublisher.Set(endPoses);

    }

    bool AreAnyCoralInClaw()
    {
        for (Coral coral : coralHolder)
        {
            if (coral.GetState() == InClaw) return true;
        }
        return false;
    }

private:
    std::vector<Coral> coralHolder;

    nt::StructArrayPublisher<frc::Pose3d> coralPublisher = nt::NetworkTableInstance::GetDefault().GetTable("SimRobot")->GetStructArrayTopic<frc::Pose3d>("coralHolder").Publish();
    nt::StructArrayPublisher<frc::Pose3d> coralEndsPublisher = nt::NetworkTableInstance::GetDefault().GetTable("SimRobot")->GetStructArrayTopic<frc::Pose3d>("coralEnds").Publish();
};