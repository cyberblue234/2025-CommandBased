#pragma once

#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configurator.hpp>
#include <ctre/phoenix6/swerve/SwerveDrivetrain.hpp>
#include <ctre/phoenix6/swerve/SwerveRequest.hpp>
#include <ctre/phoenix6/Utils.hpp>
#include <ctre/phoenix6/SignalLogger.hpp>

#include <frc/Notifier.h>

#include <frc2/command/sysid/SysIdRoutine.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <pathplanner/lib/util/PathPlannerLogging.h>

#include <frc/DriverStation.h>

#include <frc/smartdashboard/Field2d.h>

#include "subsystems/Limelight.h"
#include "Constants.h"


using namespace ctre::phoenix6;
using namespace pathplanner;
using namespace DrivetrainConstants;


class Drivetrain : public swerve::SwerveDrivetrain<hardware::TalonFX, hardware::TalonFX, hardware::CANcoder>, public frc2::SubsystemBase
{
public:
    Drivetrain();

    frc2::CommandPtr DriveWithSpeedsCommand(std::function<frc::ChassisSpeeds()> speedsSupplier, bool fieldCentric);

    frc2::CommandPtr DriveWithSpeedsAtAngleCommand(std::function<frc::ChassisSpeeds()> speedsSupplier, frc::Rotation2d rotation);

    frc2::CommandPtr ApplyRobotSpeedsCommand(std::function<frc::ChassisSpeeds()> speedsSupplier);

    frc2::CommandPtr BrakeCommand()
    {
        return RunOnce
        (
            [this]
            {
                SetControl(brake);
            }
        );
    }

    frc2::CommandPtr PathfindToBranch(int tid, Sides side, units::meter_t offset, bool usePPLibPathfinding)
    {
        if (tid < 6 || (tid > 11 && tid < 17) || tid > 22) return BrakeCommand();
        frc::Pose2d aprilTagPose = aprilTagFieldLayout.GetTagPose(tid)->ToPose2d();

        units::degree_t theta = aprilTagPose.Rotation().Degrees() + 90_deg;
        units::meter_t deltaX1 = units::math::cos(theta) * FieldConstants::kDeltaReefAprilTagToBranch;
        units::meter_t deltaY1 = units::math::sin(theta) * FieldConstants::kDeltaReefAprilTagToBranch;
        if (tid < 9 || (tid > 16 && tid < 20) == false)
        {
            if (side == Sides::Right)
            {
                deltaX1 = -deltaX1;
                deltaY1 = -deltaY1;
            }
        }
        else
        {
            if (side == Sides::Left)
            {
                deltaX1 = -deltaX1;
                deltaY1 = -deltaY1;
            }
        }
        
        units::meter_t deltaX2 = units::math::sin(theta) * offset;
        units::meter_t deltaY2 = -units::math::cos(theta) * offset;

        frc::Pose2d pose{aprilTagPose.X() + deltaX1 + deltaX2, aprilTagPose.Y() + deltaY1 + deltaY2, aprilTagPose.Rotation().Degrees() + 180_deg};
        // return DriveToPose(pose, pose.Rotation(), frc::TrajectoryConfig{kReefPathfindingConstraints.getMaxVelocity(), kReefPathfindingConstraints.getMaxAcceleration()});
        // Uses PPLib pathfinding with given constraints
        if (usePPLibPathfinding) return AutoBuilder::pathfindToPose(pose, PathPlannerConstants::kReefPathfindingConstraints);
        // Uses internal pathfinding
        return PathfindToPose(pose, pose.Rotation(), true, PathPlannerConstants::kReefPathfindingConstraints);
    }

    frc2::CommandPtr PathfindToPose(frc::Pose2d pose, frc::Rotation2d endHeading, bool preventFlipping, PathConstraints pathConstraints)
    {
        // Finds the difference of the two x and the two y values
        double xDiff = pose.X().value() - GetState().Pose.X().value();
        double yDiff = pose.Y().value() - GetState().Pose.Y().value();
        // cos(x) is equal to the lengh of the adjacent side divided by the length of the hypotenuse
        // The inverse of cos will give you the angle to drive at, in quadrants one and two
        // If you multiply that by the sign of the yDiff, you will get the final heading in radians
        units::radian_t heading = units::radian_t(sgn(yDiff) * acos((xDiff) / (pow(pow(xDiff, 2) + pow(yDiff, 2), 0.5))));
        // Creates a vector of two poses with the rotation being the heading to drive at
        // The first pose is the current pose, and the second is the pose to drive to
        std::vector<frc::Pose2d> poses 
        {
            frc::Pose2d(GetState().Pose.X(), GetState().Pose.Y(), frc::Rotation2d(heading)), 
            frc::Pose2d(pose.X(), pose.Y(), endHeading)
        };
        // Creates a path based on the vector of poses and PathPlanner constraints
        auto path = std::make_shared<PathPlannerPath>(
            PathPlannerPath::waypointsFromPoses(poses),
            pathConstraints,
            std::nullopt,
            GoalEndState(0.0_mps, pose.Rotation())
        );
        // If preventFlipping is true, it stops the path from flipping automatically 
        // because we have already flipped the desired poses
        path->preventFlipping = preventFlipping;
        // Creates and returns the command to follow the path
        return AutoBuilder::followPath(path);
    }

    frc2::CommandPtr ScheduleSysIdDynamic(frc2::sysid::Direction direction)
    {
        return frc2::cmd::RunOnce
        (
            [this, direction] ()
            {
                sysIdRoutineCommand = sysIdRoutineTranslation.Dynamic(direction);
                if (sysIdRoutineToApply == SysIdRoutines::Steer)
                    sysIdRoutineCommand = sysIdRoutineSteer.Dynamic(direction);
                else if (sysIdRoutineToApply == SysIdRoutines::Rotation)
                    sysIdRoutineCommand = sysIdRoutineRotation.Dynamic(direction);
                sysIdRoutineCommand->Schedule();
            }
        );
    }
    frc2::CommandPtr ScheduleSysIdQuasistatic(frc2::sysid::Direction direction)
    {
        return frc2::cmd::RunOnce
        (
            [this, direction] ()
            {
                sysIdRoutineCommand = sysIdRoutineTranslation.Quasistatic(direction);
                if (sysIdRoutineToApply == SysIdRoutines::Steer)
                    sysIdRoutineCommand = sysIdRoutineSteer.Quasistatic(direction);
                else if (sysIdRoutineToApply == SysIdRoutines::Rotation)
                    sysIdRoutineCommand = sysIdRoutineRotation.Quasistatic(direction);
                sysIdRoutineCommand->Schedule();
            }
        );
    }

    frc2::CommandPtr CancelSysId()
    {
        return frc2::cmd::RunOnce
        (
            [this] ()
            {
                if (sysIdRoutineCommand)
                    sysIdRoutineCommand->Cancel();
            }
        );
    }

    void Periodic() override;

    void InitSendable(wpi::SendableBuilder &builder) override;

    frc::Field2d *GetField() { return &field; }

    frc2::sysid::SysIdRoutine &GetTranslationRoutine() { return sysIdRoutineTranslation; }
    frc2::sysid::SysIdRoutine &GetSteerRoutine() { return sysIdRoutineSteer; }
    frc2::sysid::SysIdRoutine &GetRotationRoutine() { return sysIdRoutineRotation; }

    enum SysIdRoutines
    {
        Translation, Rotation, Steer
    };

    void SetSysIdRoutineToApply(const SysIdRoutines &routine)
    {        
        sysIdRoutineToApply = routine;
    }
    
    const SysIdRoutines &GetSysIdRoutineToApply()
    {
        return sysIdRoutineToApply;
    }

    std::optional<std::function<std::vector<PoseEstimate>()>> limelightPoseEstimatesSupplier;
    wpi::array<double, 3> visionStdDevs{1.0, 1.0, 9999.0};
    
private:
    bool hasAppliedDriverPerspective = false;

    swerve::requests::FieldCentric driveFieldCentric = swerve::requests::FieldCentric() 
        .WithDriveRequestType(swerve::impl::DriveRequestType::Velocity)
        .WithDeadband(kMaxSpeed * 0.15).WithRotationalDeadband(kMaxAngularSpeed * 0.10);
    swerve::requests::FieldCentricFacingAngle driveFieldCentricAtAngle = swerve::requests::FieldCentricFacingAngle()
        .WithDriveRequestType(swerve::impl::DriveRequestType::Velocity)
        .WithDeadband(kMaxSpeed * 0.15)
        .WithHeadingPID(PathPlannerConstants::Rotation::kP, PathPlannerConstants::Rotation::kI, PathPlannerConstants::Rotation::kD);
    swerve::requests::RobotCentric driveRobotCentric = swerve::requests::RobotCentric() 
        .WithDriveRequestType(swerve::impl::DriveRequestType::Velocity)
        .WithDeadband(kMaxSpeed * 0.15).WithRotationalDeadband(kMaxAngularSpeed * 0.10);
    swerve::requests::RobotCentricFacingAngle driveRobotCentricAtAngle = swerve::requests::RobotCentricFacingAngle()
        .WithDriveRequestType(swerve::impl::DriveRequestType::Velocity)
        .WithDeadband(kMaxSpeed * 0.15)
        .WithHeadingPID(PathPlannerConstants::Rotation::kP, PathPlannerConstants::Rotation::kI, PathPlannerConstants::Rotation::kD);
    swerve::requests::ApplyRobotSpeeds applyRobotSpeeds{};
    swerve::requests::SwerveDriveBrake brake{};

    std::shared_ptr<nt::NetworkTable> GetTable()
    {
        return nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard")->GetSubTable("Swerve");
    }

    nt::StructPublisher<frc::Pose2d> odometryPublisher = GetTable()->GetStructTopic<frc::Pose2d>("odometry").Publish();
    nt::StructPublisher<frc::ChassisSpeeds> speedsPublisher = GetTable()->GetStructTopic<frc::ChassisSpeeds>("speeds").Publish();
    nt::StructPublisher<frc::ChassisSpeeds> setSpeedsPublisher = GetTable()->GetStructTopic<frc::ChassisSpeeds>("setSpeeds").Publish();
    frc::ChassisSpeeds setSpeeds;
    nt::StructArrayPublisher<frc::SwerveModulePosition> modulePositionsPublisher = GetTable()->GetStructArrayTopic<frc::SwerveModulePosition>("modulePositions").Publish();
    nt::StructArrayPublisher<frc::SwerveModuleState> moduleStatesPublisher = GetTable()->GetStructArrayTopic<frc::SwerveModuleState>("moduleStates").Publish();
    nt::StructArrayPublisher<frc::SwerveModuleState> moduleTargetsPublisher = GetTable()->GetStructArrayTopic<frc::SwerveModuleState>("moduleTargets").Publish();

    std::optional<frc2::CommandPtr> sysIdRoutineCommand;
    
    swerve::requests::SysIdSwerveTranslation translationCharacterization{};
    swerve::requests::SysIdSwerveSteerGains steerCharacterization{};
    swerve::requests::SysIdSwerveRotation rotationCharacterization{};

    frc2::sysid::SysIdRoutine sysIdRoutineTranslation
    {
        frc2::sysid::Config
        {
            {},
            4_V,
            {},
            [this] (frc::sysid::State state) 
            { 
                SignalLogger::WriteString("SysIdTranslation_State", frc::sysid::SysIdRoutineLog::StateEnumToString(state));
            }
        },
        frc2::sysid::Mechanism
        {
            [this](units::volt_t volts) { SetControl(translationCharacterization.WithVolts(volts)); },
            [](auto) {},
            this
        }
    };
    frc2::sysid::SysIdRoutine sysIdRoutineSteer
    {
        frc2::sysid::Config
        {
            {},
            7_V,
            {},
            [this] (frc::sysid::State state) 
            { 
                SignalLogger::WriteString("SysIdSteer_State", frc::sysid::SysIdRoutineLog::StateEnumToString(state));
            }
        },
        frc2::sysid::Mechanism
        {
            [this](units::volt_t volts) { SetControl(steerCharacterization.WithVolts(volts)); },
            [](auto) {},
            this
        }
    };
    frc2::sysid::SysIdRoutine sysIdRoutineRotation
    {
        frc2::sysid::Config
        {
            units::volt_t{std::numbers::pi / 6} / 1_s,
            units::volt_t{std::numbers::pi},
            {},
            [this] (frc::sysid::State state) 
            { 
                SignalLogger::WriteString("SysIdRotation_State", frc::sysid::SysIdRoutineLog::StateEnumToString(state));
            }
        },
        frc2::sysid::Mechanism
        {
            [this](units::volt_t volts) 
            {
                SetControl(rotationCharacterization.WithRotationalRate(units::radians_per_second_t{volts.value()})); 
                SignalLogger::WriteDouble("Rotational_Rate", volts.value());
            },
            [](auto) {},
            this
        }
    };

    SysIdRoutines sysIdRoutineToApply = SysIdRoutines::Translation;

    std::string SysIdRoutineToString()
    {
        switch(sysIdRoutineToApply)
        {
            case SysIdRoutines::Translation:
                return "Translation";
                break;
            case SysIdRoutines::Rotation:
                return "Rotation";
                break;
            case SysIdRoutines::Steer:
                return "Steer";
                break;
            default:
                return "";
                break;
        }
    }

    const PIDConstants translationPIDs{PathPlannerConstants::Translation::kP, PathPlannerConstants::Translation::kI, PathPlannerConstants::Translation::kD};
    const PIDConstants rotationPIDs{PathPlannerConstants::Rotation::kP, PathPlannerConstants::Rotation::kI, PathPlannerConstants::Rotation::kD};

    units::second_t lastSimTime;

    frc::Field2d field;
};