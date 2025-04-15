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

#include "Constants.h"

#include <frc/smartdashboard/Field2d.h>

using namespace ctre::phoenix6;
using namespace pathplanner;
using namespace DrivetrainConstants;


class Drivetrain : public swerve::SwerveDrivetrain<hardware::TalonFX, hardware::TalonFX, hardware::CANcoder>, public frc2::SubsystemBase
{
public:
    Drivetrain();

    frc2::CommandPtr ApplyRequest(std::function<std::shared_ptr<swerve::requests::SwerveRequest>()> requestSupplier)
    {
        return Run([this, requestSupplier] 
            { 
                persistentRequest = requestSupplier();
                SetControl(*persistentRequest);
            }).WithName("SwerveApplyRequest");
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

        if (utils::IsSimulation())
        {
            const units::second_t currentTime = utils::GetCurrentTime();
            units::second_t deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            UpdateSimState(deltaTime, frc::RobotController::GetBatteryVoltage());
        }

        field.SetRobotPose(GetState().Pose);
    }
    
private:
    
    bool hasAppliedDriverPerspective = false;

    std::shared_ptr<ctre::phoenix6::swerve::requests::SwerveRequest> persistentRequest;

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