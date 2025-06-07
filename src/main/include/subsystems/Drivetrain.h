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

    frc2::CommandPtr AlignToReefCommand(std::function<std::vector<double>()> t2dSupplier, Sides side)
    {
        // [0: targetValid, 1: targetCount, 2: targetLatency, 3: captureLatency, 4: tx, 5: ty, 6: txnc, 7: tync, 8: ta, 9: tid, targetClassIndexDetector , targetClassIndexClassifier, targetLongSidePixels, targetShortSidePixels, targetHorizontalExtentPixels, targetVerticalExtentPixels, targetSkewDegrees]
        return StartRun(
        [this]
        {
            SetControl(brake);
        },
        [this, t2dSupplier, side]
        {
            std::vector<double> t2d = t2dSupplier();
            double tid = t2d[9];
            // Confirm tag ID is on the reef
            if (tid < 6 || (tid > 11 && tid < 17) || tid > 22) return;
            
        });
    }

    frc2::CommandPtr GoToPositionCommand(frc::Pose2d desiredPose)
    {
        return Run([this, desiredPose]
        {
            frc::Pose2d currentPose = GetState().Pose;
            units::meter_t deltaX = desiredPose.X() - currentPose.X();
            units::meter_t deltaY = desiredPose.Y() - currentPose.Y();
            units::radian_t deltaTheta = desiredPose.Rotation().Radians() - currentPose.Rotation().Radians();
            // Theoretical kP
            deltaX *= 0.15;
            deltaY *= 0.15;
            deltaTheta *= 0.15;
            units::meters_per_second_t vx = deltaX.value() * kMaxSpeed;
            units::meters_per_second_t vy = deltaY.value() * kMaxSpeed;
            units::radians_per_second_t omega = deltaTheta.value() * kMaxAngularSpeed;

            setSpeeds.vx = vx;
            setSpeeds.vy = vy;
            setSpeeds.omega = omega;

            SetControl(driveRobotCentric.WithVelocityX(vx).WithVelocityY(vy).WithRotationalRate(omega));
        });
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
    wpi::array<double, 3> visionStdDevs{1.0, 1.0, 1.0};
    
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