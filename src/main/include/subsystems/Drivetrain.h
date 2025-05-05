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

    frc2::CommandPtr DriveWithSpeedsCommand(std::function<frc::ChassisSpeeds()> speedsSupplier, bool fieldCentric, bool slow);

    frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction)
    {
        return sysIdRoutineToApply.Dynamic(direction);
    }
    frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction)
    {
        return sysIdRoutineToApply.Quasistatic(direction);
    }

    void Periodic() override;

    void InitSendable(wpi::SendableBuilder &builder) override
    {
        frc2::SubsystemBase::InitSendable(builder);

        builder.AddDoubleProperty("setSpeed",
            [this] 
            {
                frc::ChassisSpeeds speeds = GetState().Speeds;
                return std::pow(std::pow(speeds.vx.value(), 2) + std::pow(speeds.vy.value(), 2), 0.5);
            },
            nullptr
        );

        std::string modules[] = {"frontLeft", "frontRight", "backLeft", "backRight"};

        for (int i = 0; i < 4; i++)
        {
            builder.AddDoubleProperty(modules[i] + "/speed",
                [this, i] { return GetState().ModuleStates[i].speed.value(); },
                nullptr
            );
            builder.AddDoubleProperty(modules[i] + "/angle",
                [this, i] { return GetState().ModuleStates[i].angle.Degrees().value(); },
                nullptr
            );
            builder.AddDoubleProperty(modules[i] + "/speedSet",
                [this, i] { return GetState().ModuleTargets[i].speed.value(); },
                nullptr
            );
            builder.AddDoubleProperty(modules[i] + "/angleSet",
                [this, i] { return GetState().ModuleTargets[i].angle.Degrees().value(); },
                nullptr
            );
            builder.AddDoubleProperty(modules[i] + "/distance",
                [this, i] { return GetState().ModulePositions[i].distance.value(); },
                nullptr
            );
        }

        builder.AddStringProperty("sysIdRoutineToApply",
            [this] { return sysIdRoutineToApplyName; },
            {}
        );
    }

    frc::Field2d *GetField() { return &field; }

    frc2::sysid::SysIdRoutine &GetTranslationRoutine() { return sysIdRoutineTranslation; }
    frc2::sysid::SysIdRoutine &GetSteerRoutine() { return sysIdRoutineSteer; }
    frc2::sysid::SysIdRoutine &GetRotationRoutine() { return sysIdRoutineRotation; }

    void SetSysIdRoutineToApply(frc2::sysid::SysIdRoutine &routine, const std::string &name)
    {
        sysIdRoutineToApply = std::move(routine);
        sysIdRoutineToApplyName = name;
    }
    
    const std::string &GetSysIdRoutineToApply()
    {
        return sysIdRoutineToApplyName;
    }
    
private:
    bool hasAppliedDriverPerspective = false;

    ctre::phoenix6::swerve::requests::FieldCentric driveFieldCentric = ctre::phoenix6::swerve::requests::FieldCentric() 
        .WithDeadband(DrivetrainConstants::kMaxSpeed * 0.2).WithRotationalDeadband(DrivetrainConstants::kMaxAngularSpeed * 0.2)
        .WithDriveRequestType(ctre::phoenix6::swerve::impl::DriveRequestType::OpenLoopVoltage);
    ctre::phoenix6::swerve::requests::FieldCentric driveFieldCentricSlow = ctre::phoenix6::swerve::requests::FieldCentric() 
        .WithDriveRequestType(ctre::phoenix6::swerve::impl::DriveRequestType::OpenLoopVoltage);
    ctre::phoenix6::swerve::requests::RobotCentric driveRobotCentric = ctre::phoenix6::swerve::requests::RobotCentric() 
        .WithDeadband(DrivetrainConstants::kMaxSpeed * 0.1).WithRotationalDeadband(DrivetrainConstants::kMaxAngularSpeed * 0.1)
        .WithDriveRequestType(ctre::phoenix6::swerve::impl::DriveRequestType::OpenLoopVoltage);
    ctre::phoenix6::swerve::requests::RobotCentric driveRobotCentricSlow = ctre::phoenix6::swerve::requests::RobotCentric() 
        .WithDriveRequestType(ctre::phoenix6::swerve::impl::DriveRequestType::OpenLoopVoltage);
    ctre::phoenix6::swerve::requests::SwerveDriveBrake brake{};

    swerve::requests::ApplyRobotSpeeds pathApplyRobotSpeeds{};
    swerve::requests::SysIdSwerveTranslation translationCharacterization{};
    swerve::requests::SysIdSwerveSteerGains steerCharacterization{};
    swerve::requests::SysIdSwerveRotation rotationCharacterization{};

    std::shared_ptr<nt::NetworkTable> GetTable()
    {
        return nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard")->GetSubTable("Swerve");
    }

    nt::StructPublisher<frc::Pose2d> odometryPublisher = GetTable()->GetStructTopic<frc::Pose2d>("odometry").Publish();
    nt::StructPublisher<frc::ChassisSpeeds> speedsPublisher = GetTable()->GetStructTopic<frc::ChassisSpeeds>("speeds").Publish();
    nt::StructArrayPublisher<frc::SwerveModulePosition> modulePositionsPublisher = GetTable()->GetStructArrayTopic<frc::SwerveModulePosition>("modulePositions").Publish();
    nt::StructArrayPublisher<frc::SwerveModuleState> moduleStatesPublisher = GetTable()->GetStructArrayTopic<frc::SwerveModuleState>("moduleStates").Publish();
    nt::StructArrayPublisher<frc::SwerveModuleState> moduleTargetsPublisher = GetTable()->GetStructArrayTopic<frc::SwerveModuleState>("moduleTargets").Publish();
    
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

    frc2::sysid::SysIdRoutine sysIdRoutineToApply = std::move(sysIdRoutineTranslation);
    std::string sysIdRoutineToApplyName = "translation";

    const PIDConstants translationPIDs{PathPlannerConstants::Translation::kP, PathPlannerConstants::Translation::kI, PathPlannerConstants::Translation::kD};
    const PIDConstants rotationPIDs{PathPlannerConstants::Rotation::kP, PathPlannerConstants::Rotation::kI, PathPlannerConstants::Rotation::kD};

    // const units::second_t kSimLoopPeriod = 5_ms;
    // frc::Notifier simNotifier{[this](){}};
    units::second_t lastSimTime;

    // void StartSimThread()
    // {
    //     lastSimTime = utils::GetCurrentTime();
    //     simNotifier = frc::Notifier
    //     (
    //         [this]
    //         {
    //             const units::second_t currentTime = utils::GetCurrentTime();
    //             units::second_t deltaTime = currentTime - lastSimTime;
    //             lastSimTime = currentTime;

    //             UpdateSimState(deltaTime, frc::RobotController::GetBatteryVoltage());
    //         }
    //     );
    //     simNotifier.StartPeriodic(kSimLoopPeriod);
    // }


    frc::Field2d field;
};