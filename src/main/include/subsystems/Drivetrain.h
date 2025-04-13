#pragma once

#include <frc2/command/Subsystem.h>

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

#include "Constants.h"

#include <frc/smartdashboard/Field2d.h>

using namespace ctre::phoenix6;
using namespace pathplanner;
using namespace DrivetrainConstants;

class Drivetrain : public swerve::SwerveDrivetrain<hardware::TalonFX, hardware::TalonFX, hardware::CANcoder>, public frc2::Subsystem
{
public:
    Drivetrain();

    frc2::CommandPtr ApplyRequest(std::function<std::unique_ptr<swerve::requests::SwerveRequest>()> requestSupplier)
    {
        return frc2::cmd::Run([this, requestSupplier = std::move(requestSupplier)] { this->SetControl(*requestSupplier()); });
    }
    
    frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction)
    {
        return sysIdRoutineToApply.Dynamic(direction);
    }
    frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction)
    {
        return sysIdRoutineToApply.Quasistatic(direction);
    }

    void Periodic() override
    {
        if (!hasAppliedDriverPerspective || frc::DriverStation::IsDisabled())
        {
            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance) 
            {
                SetOperatorPerspectiveForward(alliance.value() == frc::DriverStation::Alliance::kBlue ? kBlueAlliancePersepctiveRotation : kRedAlliancePersepctiveRotation);
                hasAppliedDriverPerspective = true; 
            }
        }

        field.SetRobotPose(GetState().Pose);
    }
    
private:
    const configs::Slot0Configs driveGains = configs::Slot0Configs()
        .WithKP(Drive::kP).WithKI(Drive::kI).WithKD(Drive::kD)
        .WithKS(Drive::kS).WithKV(Drive::kV).WithKA(Drive::kA);
    const configs::Slot0Configs steerGains = configs::Slot0Configs()
        .WithKP(Steer::kP).WithKI(Steer::kI).WithKD(Steer::kD)
        .WithKS(Steer::kS).WithKV(Steer::kV).WithKA(Steer::kA)
        .WithStaticFeedforwardSign(signals::StaticFeedforwardSignValue::UseClosedLoopSign);

    const swerve::ClosedLoopOutputType kDriveClosedLoopOutput = swerve::ClosedLoopOutputType::Voltage;
    const swerve::ClosedLoopOutputType kSteerClosedLoopOutput = swerve::ClosedLoopOutputType::Voltage;

    const swerve::DriveMotorArrangement kDriveMotorType = swerve::DriveMotorArrangement::TalonFX_Integrated;
    const swerve::SteerMotorArrangement kSteerMotorType = swerve::SteerMotorArrangement::TalonFX_Integrated;
    const swerve::SteerFeedbackType kSteerFeedbackType = swerve::SteerFeedbackType::RemoteCANcoder;

    const configs::TalonFXConfiguration driveInitialConfigs = configs::TalonFXConfiguration()
        .WithCurrentLimits(configs::CurrentLimitsConfigs()
            .WithStatorCurrentLimit(Drive::kStatorCurrentLimit)
            .WithStatorCurrentLimitEnable(true));
    const configs::TalonFXConfiguration steerInitialConfigs = configs::TalonFXConfiguration()
        .WithCurrentLimits(configs::CurrentLimitsConfigs()
            .WithStatorCurrentLimit(Steer::kStatorCurrentLimit)
            .WithStatorCurrentLimitEnable(true));
    const configs::CANcoderConfiguration canCoderInitialConfigs{};
    const configs::Pigeon2Configuration pigeonConfigs{};

    const CANBus kCANBus{"rio"};

    const swerve::SwerveDrivetrainConstants drivetrainConstants = swerve::SwerveDrivetrainConstants()
        .WithCANBusName(kCANBus.GetName()).WithPigeon2Id(RobotMap::Drivetrain::kGyroID).WithPigeon2Configs(pigeonConfigs);

    const swerve::SwerveModuleConstantsFactory<configs::TalonFXConfiguration, configs::TalonFXConfiguration, configs::CANcoderConfiguration> swerveModuleConstantsCreator = 
        swerve::SwerveModuleConstantsFactory<configs::TalonFXConfiguration, configs::TalonFXConfiguration, configs::CANcoderConfiguration>()
            .WithDriveMotorGearRatio(Drive::kGearRatio).WithSteerMotorGearRatio(Steer::kGearRatio)
            .WithCouplingGearRatio(kCoupleRatio)
            .WithWheelRadius(kWheelRadius)
            .WithDriveMotorGains(driveGains).WithSteerMotorGains(steerGains)
            .WithDriveMotorClosedLoopOutput(kDriveClosedLoopOutput).WithSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
            .WithSlipCurrent(kSlipCurrent)
            .WithSpeedAt12Volts(kMaxSpeed)
            .WithDriveMotorType(kDriveMotorType).WithSteerMotorType(kSteerMotorType)
            .WithFeedbackSource(kSteerFeedbackType)
            .WithDriveMotorInitialConfigs(driveInitialConfigs).WithSteerMotorInitialConfigs(steerInitialConfigs)
            .WithEncoderInitialConfigs(canCoderInitialConfigs)
            .WithDriveInertia(Drive::kInertia).WithSteerInertia(Steer::kInertia)
            .WithDriveFrictionVoltage(Drive::kFrictionVoltage).WithSteerFrictionVoltage(Steer::kFrictionVoltage);

    const swerve::SwerveModuleConstants<configs::TalonFXConfiguration, configs::TalonFXConfiguration, configs::CANcoderConfiguration> frontLeft = 
        swerveModuleConstantsCreator.CreateModuleConstants
        (
            RobotMap::Drivetrain::FrontLeft::kDriveID, RobotMap::Drivetrain::FrontLeft::kSteerID, RobotMap::Drivetrain::FrontLeft::kCANcoderID,
            FrontLeft::kMagnetOffset, FrontLeft::kLocation.X(), FrontLeft::kLocation.Y(), 
            FrontLeft::kDriveMotorInverted, FrontLeft::kSteerMotorInverted, FrontLeft::kCANcoderInverted
        );
    const swerve::SwerveModuleConstants<configs::TalonFXConfiguration, configs::TalonFXConfiguration, configs::CANcoderConfiguration> frontRight = 
        swerveModuleConstantsCreator.CreateModuleConstants
        (
            RobotMap::Drivetrain::FrontRight::kDriveID, RobotMap::Drivetrain::FrontRight::kSteerID, RobotMap::Drivetrain::FrontRight::kCANcoderID,
            FrontRight::kMagnetOffset, FrontRight::kLocation.X(), FrontRight::kLocation.Y(), 
            FrontRight::kDriveMotorInverted, FrontRight::kSteerMotorInverted, FrontRight::kCANcoderInverted
        );
    const swerve::SwerveModuleConstants<configs::TalonFXConfiguration, configs::TalonFXConfiguration, configs::CANcoderConfiguration> backLeft = 
        swerveModuleConstantsCreator.CreateModuleConstants
        (
            RobotMap::Drivetrain::BackLeft::kDriveID, RobotMap::Drivetrain::BackLeft::kSteerID, RobotMap::Drivetrain::BackLeft::kCANcoderID,
            BackLeft::kMagnetOffset, BackLeft::kLocation.X(), BackLeft::kLocation.Y(), 
            BackLeft::kDriveMotorInverted, BackLeft::kSteerMotorInverted, BackLeft::kCANcoderInverted
        );
    const swerve::SwerveModuleConstants<configs::TalonFXConfiguration, configs::TalonFXConfiguration, configs::CANcoderConfiguration> backRight = 
        swerveModuleConstantsCreator.CreateModuleConstants
        (
            RobotMap::Drivetrain::BackRight::kDriveID, RobotMap::Drivetrain::BackRight::kSteerID, RobotMap::Drivetrain::BackRight::kCANcoderID,
            BackRight::kMagnetOffset, BackRight::kLocation.X(), BackRight::kLocation.Y(), 
            BackRight::kDriveMotorInverted, BackRight::kSteerMotorInverted, BackRight::kCANcoderInverted
        );

    const frc::Rotation2d kBlueAlliancePersepctiveRotation{0_deg};
    const frc::Rotation2d kRedAlliancePersepctiveRotation{180_deg};
    bool hasAppliedDriverPerspective = false;

    swerve::requests::ApplyRobotSpeeds pathApplyRobotSpeeds{};
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

    frc2::sysid::SysIdRoutine sysIdRoutineToApply = std::move(sysIdRoutineTranslation);

    const PIDConstants translationPIDs{PathPlannerConstants::Translation::kP, PathPlannerConstants::Translation::kI, PathPlannerConstants::Translation::kD};
    const PIDConstants rotationPIDs{PathPlannerConstants::Rotation::kP, PathPlannerConstants::Rotation::kI, PathPlannerConstants::Rotation::kD};

    const units::second_t kSimLoopPeriod = 5_ms;
    frc::Notifier simNotifier{[this](){}};
    units::second_t lastSimTime;

    void StartSimThread()
    {
        lastSimTime = utils::GetCurrentTime();
        simNotifier = frc::Notifier
        (
            [this]
            {
                const units::second_t currentTime = utils::GetCurrentTime();
                units::second_t deltaTime = currentTime - lastSimTime;
                lastSimTime = currentTime;

                UpdateSimState(deltaTime, frc::RobotController::GetBatteryVoltage());
            }
        );
        simNotifier.StartPeriodic(kSimLoopPeriod);
    }


    frc::Field2d field;
};