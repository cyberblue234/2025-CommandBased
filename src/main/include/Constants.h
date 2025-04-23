#pragma once

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/area.h>
#include <units/capacitance.h>
#include <units/charge.h>
#include <units/concentration.h>
#include <units/conductance.h>
#include <units/current.h>
#include <units/curvature.h>
#include <units/data.h>
#include <units/data_transfer_rate.h>
#include <units/density.h>
#include <units/dimensionless.h>
#include <units/energy.h>
#include <units/force.h>
#include <units/frequency.h>
#include <units/illuminance.h>
#include <units/impedance.h>
#include <units/inductance.h>
#include <units/length.h>
#include <units/luminous_flux.h>
#include <units/luminous_intensity.h>
#include <units/magnetic_field_strength.h>
#include <units/magnetic_flux.h>
#include <units/mass.h>
#include <units/moment_of_inertia.h>
#include <units/power.h>
#include <units/pressure.h>
#include <units/radiation.h>
#include <units/solid_angle.h>
#include <units/substance.h>
#include <units/temperature.h>
#include <units/time.h>
#include <units/torque.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/volume.h>

#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <pathplanner/lib/path/PathConstraints.h>
#include <pathplanner/lib/config/ModuleConfig.h>
#include <pathplanner/lib/config/RobotConfig.h>

#include <frc/RobotController.h>
#include <frc/RobotBase.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include <networktables/IntegerTopic.h>
#include <networktables/IntegerArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/StringTopic.h>
#include <networktables/StructTopic.h>
#include <networktables/StructArrayTopic.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/StartEndCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#include <ctre/phoenix6/swerve/SwerveDrivetrainConstants.hpp>
#include <ctre/phoenix6/swerve/SwerveModuleConstants.hpp>
#include <ctre/phoenix6/swerve/SwerveDrivetrain.hpp>

#include <numbers>
#include <string>
#include <array>
#include <vector>
#include <math.h>
#include <iostream>

/// @brief Contains the IDs of all the CAN devices. They all have device types to make sure no conflicts of IDs
namespace RobotMap
{
    /// @brief The Drivetrain IDs
    namespace Drivetrain
    {
        namespace FrontLeft
        {
            // TalonFX
            inline constexpr int kDriveID = 1;
            // TalonFX
            inline constexpr int kSteerID = 2;
            // CANCoder
            inline constexpr int kCANcoderID = 1;
        }
        namespace FrontRight
        {
            // TalonFX
            inline constexpr int kDriveID = 3;
            // TalonFX
            inline constexpr int kSteerID = 4;
            // CANCoder
            inline constexpr int kCANcoderID = 2;
        }
        namespace BackLeft
        {
            // TalonFX
            inline constexpr int kDriveID = 5;
            // TalonFX
            inline constexpr int kSteerID = 6;
            // CANCoder
            inline constexpr int kCANcoderID = 3;
        }
        namespace BackRight
        {
            // TalonFX
            inline constexpr int kDriveID = 7;
            // TalonFX
            inline constexpr int kSteerID = 8;
            // CANCoder
            inline constexpr int kCANcoderID = 4;
        }

        // Pigeon2
        inline constexpr int kGyroID = 1;
    }

    /// @brief The Elevator IDs
    namespace Elevator
    {
        //TalonFX
        inline constexpr int kMotor1ID = 9;
        //TalonFX
        inline constexpr int kMotor2ID = 10;
        //DIO
        inline constexpr int kBottomLimitSwitchID = 2;
        //DIO
        inline constexpr int kTopLimitSwitchID = 3;
    }

    namespace Claw
    {
        //TalonFX
        inline constexpr int kWristMotorID = 11;
        //SparkFlex
        inline constexpr int kIOMotorID = 12;
        //CANcoder
        inline constexpr int kCanCoderID = 5;

        //CANrange
        inline constexpr int kCanRangeID = 1;
    }

    namespace Climber
    {
        //SparkMax
        inline constexpr int kClimbMotorID = 2;
        //DIO
        inline constexpr int kLimitSwitch = 1;
    }
    namespace Pneumatics
    {
        //RevPH
        inline constexpr int kPneumaticHubID = 1;
        //Pneumatic Hub Slot
        inline constexpr int kStopperSlot = 0;
    }
}

namespace ControlsConstants
{
    // Analog inputs
    constexpr int kManualWristAxis = 2;
    constexpr int kManualElevatorAxis = 1;
    constexpr int kClimberAxis = 3;
    constexpr int kManualIntakeAxis = 0;

    constexpr int kL1Button = 5;
    constexpr int kL2Button = 3;
    constexpr int kL3Button = 1;
    constexpr int kL4Button = 6;
    constexpr int kAlgaeHighButton = 8;
    constexpr int kAlgaeLowButton = 11;
    constexpr int kCoralStationButton = 4;
    constexpr int kProcessorButton = 7;
    constexpr int kBargeButton = 2;
    constexpr int kCoralHomeButton = 9;
    constexpr int kAlgaeHomeButton = 12;

    constexpr int kIOButton = 10;
}

/// @brief Personal add-on to the WPILib units library. See https://docs.wpilib.org/en/stable/docs/software/basic-programming/cpp-units.html for details on it.
namespace units
{
    using meters_per_turn = compound_unit<meter, inverse<turn>>;
    using meters_per_turn_t = unit_t<meters_per_turn>;
    using radians_per_turn = compound_unit<radian, inverse<turn>>;
    using radians_per_turn_t = unit_t<radians_per_turn>;
    using degrees_per_turn = compound_unit<degree, inverse<turn>>;
    using degrees_per_turn_t = unit_t<degrees_per_turn>;
}

namespace RobotConstants
{
    inline constexpr units::meter_t kRobotWidth = 28_in;
    inline constexpr units::meter_t kRobotLength = 31_in;
    inline constexpr units::meter_t kBumperWidth = 3_in;
    inline constexpr units::meter_t kRobotWidthWithBumper = kRobotWidth + kBumperWidth * 2;
    inline constexpr units::meter_t kRobotLengthWithBumper = kRobotLength + kBumperWidth * 2;
    inline constexpr units::meter_t kTrackWidth = 20.75_in;
    inline constexpr units::meter_t kWheelBase = 24_in;

    inline constexpr units::kilogram_t kBatteryMass = 12.89_lb;
    inline constexpr units::kilogram_t kBumperMass = 20.0_lb;
    inline constexpr units::kilogram_t kMass = 115_lb + kBumperMass + kBatteryMass; 
    inline constexpr units::kilogram_square_meter_t kMOI = (1.0 / 12.0) * kMass * (kRobotWidthWithBumper * kRobotWidthWithBumper + kRobotLengthWithBumper * kRobotLengthWithBumper);
}

namespace DrivetrainConstants
{
    namespace FrontLeft
    {   
        // The locations of the swerve modules in reference to the center of the robot
        // A common error is to use an incorrect coordinate system where the positive Y axis points forward on the robot. 
        // The correct coordinate system has the positive X axis pointing forward.
        constexpr frc::Translation2d kLocation  = frc::Translation2d{+RobotConstants::kWheelBase / 2, +RobotConstants::kTrackWidth / 2};
        // Offsets for the CANcoders to ensure 0 is pointing forward
        constexpr units::turn_t kMagnetOffset  = -0.69067_tr;
        constexpr bool kDriveMotorInverted = false;
        constexpr bool kSteerMotorInverted = true;
        constexpr bool kCANcoderInverted = false;
    }
    namespace FrontRight
    {   
        constexpr frc::Translation2d kLocation  = frc::Translation2d{+RobotConstants::kWheelBase / 2, -RobotConstants::kTrackWidth / 2};
        constexpr units::turn_t kMagnetOffset  = 0.68213_tr;
        constexpr bool kDriveMotorInverted = false;
        constexpr bool kSteerMotorInverted = true;
        constexpr bool kCANcoderInverted = false;
    }
    namespace BackLeft
    {   
        constexpr frc::Translation2d kLocation  = frc::Translation2d{-RobotConstants::kWheelBase / 2, +RobotConstants::kTrackWidth / 2};
        constexpr units::turn_t kMagnetOffset  = -0.052002_tr;
        constexpr bool kDriveMotorInverted = false;
        constexpr bool kSteerMotorInverted = true;
        constexpr bool kCANcoderInverted = false;
    }
    namespace BackRight
    {   
        constexpr frc::Translation2d kLocation  = frc::Translation2d{-RobotConstants::kWheelBase / 2, -RobotConstants::kTrackWidth / 2};
        constexpr units::turn_t kMagnetOffset  = -0.12573_tr;
        constexpr bool kDriveMotorInverted = false;
        constexpr bool kSteerMotorInverted = true;
        constexpr bool kCANcoderInverted = false;
    }

    // Maximum desired speed of the robot. Does not have to be maximum theoretical speed if that is too high for desired driving speed
    constexpr units::meters_per_second_t kMaxSpeed = 5.0_mps;
    // Maximum desired angular speed of the robot. Does not have to be maximum theoretical speed if that is too high for desired driving speed
    constexpr units::radians_per_second_t kMaxAngularSpeed = 900_deg_per_s;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    constexpr double kCoupleRatio = 3.8181818181818183;
    // Radius of the wheel
    constexpr units::meter_t kWheelRadius = 2_in;
    // Because we don't know the actual coeffient of friction between the wheel tread and the carpet, we just assume it is 1
    // If wheels are slipping a lot, we can lower to force pathplanner to slow down 
    constexpr double kWheelCOF = 1.0;
    constexpr units::ampere_t kSlipCurrent = 100_A;

    // PIDs and feedforward constants of the drive motor
    namespace Drive
    {
        // Gearing between the drive motor and wheel in turns - how many turns of the drive motor does it take to drive the wheel one full revolution
        constexpr double kGearRatio = 6.39;

        constexpr double kP = 0.1;
        constexpr double kI = 0.0;
        constexpr double kD = 0.0;

        constexpr double kS = 0.24;
        constexpr double kV = 5;
        constexpr double kA = 0.20;

        // This stator current limit ensures we don't damage the motor by pushing too much current
        constexpr units::ampere_t kStatorCurrentLimit = 100_A;

        // The amount of meters the robot drives per turn of the drive motor. The circumference of the wheel is the distance the robot drives for one full revolution of the wheel. Dividing by the gear ratio gets you to meters per one turn of the drive motor
        // constexpr units::meters_per_turn_t kDistanceRatio = kWheelRadius * 2 * std::numbers::pi / units::turn_t(Drive::kGearRatio);

        // Simulation values
        constexpr units::kilogram_square_meter_t kInertia{0.01}; // Moment of inertia of driving the wheels
        constexpr units::volt_t kFrictionVoltage = 0.2_V; // Simulated voltage necessary to overcome friction
    }
    // PIDs of the steer motor
    namespace Steer
    {
        // Gearing between the steer motor and wheel in turns - how many turns of the steer motor does it take to turn the wheel one full revolution
        constexpr double kGearRatio = 12.1;

        constexpr double kP = 100.0;
        constexpr double kI = 0.0;
        constexpr double kD = 0.5;

        constexpr double kS = 0.24;
        constexpr double kV = 2.3;
        constexpr double kA = 0.20;
        
        // Swerve azimuth does not require much torque output, so we can set a relatively low
        // stator current limit to help avoid brownouts without impacting performance.
        constexpr units::ampere_t kStatorCurrentLimit = 60_A;

        // Because we use the CANcoder to determine turning distance, the ratio is just one revolution of the wheel (360° or 2π) for every turn
        constexpr units::radians_per_turn_t kDistanceRatio = 2_rad * std::numbers::pi / 1_tr;

        // Simulation values
        constexpr units::kilogram_square_meter_t kInertia{0.01}; // Moment of inertia of driving the wheels
        constexpr units::volt_t kFrictionVoltage = 0.2_V; // Simulated voltage necessary to overcome friction
    }

    constexpr ctre::phoenix6::configs::Slot0Configs driveGains = ctre::phoenix6::configs::Slot0Configs()
        .WithKP(Drive::kP).WithKI(Drive::kI).WithKD(Drive::kD)
        .WithKS(Drive::kS).WithKV(Drive::kV).WithKA(Drive::kA);
    constexpr ctre::phoenix6::configs::Slot0Configs steerGains = ctre::phoenix6::configs::Slot0Configs()
        .WithKP(Steer::kP).WithKI(Steer::kI).WithKD(Steer::kD)
        .WithKS(Steer::kS).WithKV(Steer::kV).WithKA(Steer::kA)
        .WithStaticFeedforwardSign(ctre::phoenix6::signals::StaticFeedforwardSignValue::UseClosedLoopSign);

    constexpr ctre::phoenix6::swerve::ClosedLoopOutputType kDriveClosedLoopOutput = ctre::phoenix6::swerve::ClosedLoopOutputType::Voltage;
    constexpr ctre::phoenix6::swerve::ClosedLoopOutputType kSteerClosedLoopOutput = ctre::phoenix6::swerve::ClosedLoopOutputType::Voltage;

    constexpr ctre::phoenix6::swerve::DriveMotorArrangement kDriveMotorType = ctre::phoenix6::swerve::DriveMotorArrangement::TalonFX_Integrated;
    constexpr ctre::phoenix6::swerve::SteerMotorArrangement kSteerMotorType = ctre::phoenix6::swerve::SteerMotorArrangement::TalonFX_Integrated;
    constexpr ctre::phoenix6::swerve::SteerFeedbackType kSteerFeedbackType = ctre::phoenix6::swerve::SteerFeedbackType::RemoteCANcoder;

    constexpr ctre::phoenix6::configs::TalonFXConfiguration driveInitialConfigs = ctre::phoenix6::configs::TalonFXConfiguration()
        .WithCurrentLimits(ctre::phoenix6::configs::CurrentLimitsConfigs()
            .WithStatorCurrentLimit(Drive::kStatorCurrentLimit)
            .WithStatorCurrentLimitEnable(true));
    constexpr ctre::phoenix6::configs::TalonFXConfiguration steerInitialConfigs = ctre::phoenix6::configs::TalonFXConfiguration()
        .WithCurrentLimits(ctre::phoenix6::configs::CurrentLimitsConfigs()
            .WithStatorCurrentLimit(Steer::kStatorCurrentLimit)
            .WithStatorCurrentLimitEnable(true));
    constexpr ctre::phoenix6::configs::CANcoderConfiguration canCoderInitialConfigs{};
    constexpr ctre::phoenix6::configs::Pigeon2Configuration pigeonConfigs{};

    constexpr ctre::phoenix6::swerve::SwerveDrivetrainConstants drivetrainConstants = ctre::phoenix6::swerve::SwerveDrivetrainConstants()
        .WithCANBusName("rio").WithPigeon2Id(RobotMap::Drivetrain::kGyroID).WithPigeon2Configs(pigeonConfigs);

    constexpr ctre::phoenix6::swerve::SwerveModuleConstantsFactory<ctre::phoenix6::configs::TalonFXConfiguration, ctre::phoenix6::configs::TalonFXConfiguration, ctre::phoenix6::configs::CANcoderConfiguration> swerveModuleConstantsCreator = 
        ctre::phoenix6::swerve::SwerveModuleConstantsFactory<ctre::phoenix6::configs::TalonFXConfiguration, ctre::phoenix6::configs::TalonFXConfiguration, ctre::phoenix6::configs::CANcoderConfiguration>()
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

    constexpr ctre::phoenix6::swerve::SwerveModuleConstants<ctre::phoenix6::configs::TalonFXConfiguration, ctre::phoenix6::configs::TalonFXConfiguration, ctre::phoenix6::configs::CANcoderConfiguration> frontLeft = 
        swerveModuleConstantsCreator.CreateModuleConstants
        (
            RobotMap::Drivetrain::FrontLeft::kDriveID, RobotMap::Drivetrain::FrontLeft::kSteerID, RobotMap::Drivetrain::FrontLeft::kCANcoderID,
            FrontLeft::kMagnetOffset, FrontLeft::kLocation.X(), FrontLeft::kLocation.Y(), 
            FrontLeft::kDriveMotorInverted, FrontLeft::kSteerMotorInverted, FrontLeft::kCANcoderInverted
        );
    constexpr ctre::phoenix6::swerve::SwerveModuleConstants<ctre::phoenix6::configs::TalonFXConfiguration, ctre::phoenix6::configs::TalonFXConfiguration, ctre::phoenix6::configs::CANcoderConfiguration> frontRight = 
        swerveModuleConstantsCreator.CreateModuleConstants
        (
            RobotMap::Drivetrain::FrontRight::kDriveID, RobotMap::Drivetrain::FrontRight::kSteerID, RobotMap::Drivetrain::FrontRight::kCANcoderID,
            FrontRight::kMagnetOffset, FrontRight::kLocation.X(), FrontRight::kLocation.Y(), 
            FrontRight::kDriveMotorInverted, FrontRight::kSteerMotorInverted, FrontRight::kCANcoderInverted
        );
    constexpr ctre::phoenix6::swerve::SwerveModuleConstants<ctre::phoenix6::configs::TalonFXConfiguration, ctre::phoenix6::configs::TalonFXConfiguration, ctre::phoenix6::configs::CANcoderConfiguration> backLeft = 
        swerveModuleConstantsCreator.CreateModuleConstants
        (
            RobotMap::Drivetrain::BackLeft::kDriveID, RobotMap::Drivetrain::BackLeft::kSteerID, RobotMap::Drivetrain::BackLeft::kCANcoderID,
            BackLeft::kMagnetOffset, BackLeft::kLocation.X(), BackLeft::kLocation.Y(), 
            BackLeft::kDriveMotorInverted, BackLeft::kSteerMotorInverted, BackLeft::kCANcoderInverted
        );
    constexpr ctre::phoenix6::swerve::SwerveModuleConstants<ctre::phoenix6::configs::TalonFXConfiguration, ctre::phoenix6::configs::TalonFXConfiguration, ctre::phoenix6::configs::CANcoderConfiguration> backRight = 
        swerveModuleConstantsCreator.CreateModuleConstants
        (
            RobotMap::Drivetrain::BackRight::kDriveID, RobotMap::Drivetrain::BackRight::kSteerID, RobotMap::Drivetrain::BackRight::kCANcoderID,
            BackRight::kMagnetOffset, BackRight::kLocation.X(), BackRight::kLocation.Y(), 
            BackRight::kDriveMotorInverted, BackRight::kSteerMotorInverted, BackRight::kCANcoderInverted
        );

    constexpr frc::Rotation2d kBlueAlliancePersepctiveRotation{0_deg};
    constexpr frc::Rotation2d kRedAlliancePersepctiveRotation{180_deg};

    inline pathplanner::ModuleConfig moduleConfigs{kWheelRadius, DrivetrainConstants::kMaxSpeed, kWheelCOF, frc::DCMotor::KrakenX60(1), Drive::kGearRatio, Drive::kStatorCurrentLimit, 1};
}

/// @brief Constants for PathPlanner
namespace PathPlannerConstants
{
    // PIDs for the translation component of PathPlanner
    namespace Translation
    {
        constexpr double kP = 8.0;
        constexpr double kI = 0.0;
        constexpr double kD = 0.1;
    }
    // PIDs for the rotation component of PathPlanner
    namespace Rotation
    {
        constexpr double kP = 5.0;
        constexpr double kI = 0.0;
        constexpr double kD = 0.1;
    }
    
    inline pathplanner::RobotConfig kConfig{RobotConstants::kMass, RobotConstants::kMOI, DrivetrainConstants::moduleConfigs, std::vector<frc::Translation2d>{DrivetrainConstants::FrontLeft::kLocation, DrivetrainConstants::FrontRight::kLocation, DrivetrainConstants::BackLeft::kLocation, DrivetrainConstants::BackRight::kLocation}};
}


/// @brief Constants for the Elevator Class
namespace ElevatorConstants
{
    // The gearing between the motor and the sprocket
    constexpr double kMotorGearing = 20;
    // Theoretical diameter of the sprocket connected to the chain which raises the first stage
    constexpr units::meter_t kSprocketPitchDiameter = 1.751_in;
    // How many meters the carriage travels per rotation of the motors.
    // Note, carriage raises at a rate of 2:1 compared to the first stage
    constexpr units::meters_per_turn_t kMetersPerMotorTurn = 2 * (kSprocketPitchDiameter * std::numbers::pi) / units::turn_t{kMotorGearing};

    // PIDs and feedforward values
    namespace Gains
    {
        constexpr double kP = 1.3;
        constexpr double kI = 0.0;
        constexpr double kD = 0.01;
        constexpr double kS = 0.0;
        constexpr double kG = 0.0;
        constexpr double kV = 0.135;
        constexpr double kA = 0.005;
    }
    namespace MotionMagic
    {
        constexpr units::turns_per_second_t kCruiseVelocity = 200_tps;
        constexpr units::turns_per_second_squared_t kAcceleration = 400_tr_per_s_sq;
        constexpr units::turns_per_second_cubed_t kJerk = 4000_tr_per_s_cu;
    }

    // The distance from the ground to the bottom of the carriage
    constexpr units::meter_t kHeightOffset = 6.5_in;
    // Should be equal to kMaxEncoderValue * kMetersPerMotorTurn + kHeightOffset
    constexpr units::meter_t kMaxElevatorHeight = 4.5_ft + kHeightOffset;
    // Maximum encoder count - should be slightly lower than the maximum possible encoder count
    constexpr units::turn_t kMaxEncoderValue = (kMaxElevatorHeight - kHeightOffset) / kMetersPerMotorTurn;

    constexpr units::meter_t kTolerance = 0.5_in;

    constexpr double kElevatorPower = 0.25;
}

namespace ClawConstants
{
    namespace Gains
    {
        constexpr double kP = 100;
        constexpr double kI = 0.0;
        constexpr double kD = 0.0;
        constexpr double kS = 0.0;
        constexpr double kG = 0.0;
        constexpr double kV = 33;
        constexpr double kA = 0.5;
    }
    namespace MotionMagic
    {
        constexpr units::turns_per_second_t kCruiseVelocity = 0.5_tps;
        constexpr units::turns_per_second_squared_t kAcceleration = 2_tr_per_s_sq;
        constexpr units::turns_per_second_cubed_t kJerk = 6_tr_per_s_cu;
    }

    // Intake and output powers for coral and algae
    constexpr double kCoralIntakePower = -0.35;
    constexpr double kAlgaeIntakePower = 0.5;
    // Outputting should be negative compared to intaking
    constexpr double kCoralOutputPower = 0.5;
    constexpr double kProcessorPower = -0.3;
    constexpr double kBargePower = -1.0;
    constexpr double kManualIOPower = 0.2;

    constexpr units::turn_t kCanCoderMagnetOffset = 0.1582_tr;

    constexpr units::degree_t kTolerance = 2.0_deg;

    constexpr units::degree_t kLowLimit = -10.0_deg;
    constexpr units::degree_t kHighLimit = 180_deg;

    constexpr double kWristGearRatio = 233.45;
    constexpr units::degrees_per_turn_t kDegreesPerMotorTurn = 360_deg / units::turn_t{kWristGearRatio};

    constexpr double kWristPower = 0.2;
    
    constexpr units::meter_t kProximityThreshold = 2.0_in;
}

namespace ClimberConstants
{
    const double kClimberPower = 0.4;
}

namespace LimelightConstants
{
    // Forward, Right, Up, Roll, Pitch, Yaw
    const frc::Pose3d kHighOffset{6.75_in, 12_in, 38.125_in, frc::Rotation3d{0_deg, 0_deg, -2.29_deg}};
    const frc::Pose3d kLowOffset{11.25_in, -0.125_in, 7.625_in, frc::Rotation3d{0_deg, 20_deg, 0_deg}};

    const wpi::array<double, 3> autonStdDevs{1.8, 1.8, 1.8};
    const wpi::array<double, 3> teleopStdDevs{0.9, 0.9, 0.9};
}

/// @brief Struct for the different possible positions
struct Position
{
    const std::string name = "null";
    /// @brief The height of the elevator
    const units::meter_t height = 0_m;
    /// @brief The angle of the claw
    const units::degree_t angle = 0_deg;
    /// @brief What power to set to the IO motor
    const double ioMotorPower = 0.0;
    /// @brief Set to true when intaking coral - will be used to stop the IO motor when we have a coral in the claw
    const bool isForCoralIntake = false;

    const int button = -1;

    const Position& operator=(const Position &rhs)
    {
        return {rhs.name, rhs.height, rhs.angle, rhs.ioMotorPower, rhs.isForCoralIntake, rhs.button};
    }
    bool operator==(const Position &rhs)
    {
        return this->name == rhs.name;
    }
    std::string to_string() const
    {
        return name + "; Height: " + units::to_string(height.convert<units::feet>()) + "; Angle: " + units::to_string(angle) + "; IO Power: " + std::to_string(ioMotorPower);
    }
};

namespace Positions
{    
    const Position L1           = Position("L1", 1.6_ft, 150.0_deg,  ClawConstants::kCoralOutputPower, false, ControlsConstants::kL1Button);
    const Position L2           = Position("L2", 1.1_ft, 13.5_deg,  -ClawConstants::kCoralOutputPower, false, ControlsConstants::kL2Button);
    const Position L3           = Position("L3", 2.5_ft, 13.5_deg,  -ClawConstants::kCoralOutputPower, false, ControlsConstants::kL3Button);
    const Position L4           = Position("L4", 4.55_ft, 13.5_deg, -ClawConstants::kCoralOutputPower, false, ControlsConstants::kL4Button);
    const Position AlgaeLow     = Position("AlgaeLow", 2.2_ft, 170.0_deg,  ClawConstants::kAlgaeIntakePower, false, ControlsConstants::kAlgaeLowButton);
    const Position AlgaeHigh    = Position("AlgaeHigh", 3.4_ft, 170.0_deg,  ClawConstants::kAlgaeIntakePower, false, ControlsConstants::kAlgaeHighButton);
    const Position CoralStation = Position("CoralStation", 1.915_ft, 110.0_deg,  ClawConstants::kCoralIntakePower, true, ControlsConstants::kCoralStationButton);
    const Position Processor    = Position("Processor", ElevatorConstants::kHeightOffset, 160.0_deg,  ClawConstants::kProcessorPower, false, ControlsConstants::kProcessorButton);
    const Position Barge        = Position("Barge", ElevatorConstants::kMaxElevatorHeight, 63.0_deg,  ClawConstants::kBargePower, false, ControlsConstants::kBargeButton);
    const Position CoralHome    = Position("CoralHome", ElevatorConstants::kHeightOffset, 15.0_deg,  ClawConstants::kCoralIntakePower, true, ControlsConstants::kCoralHomeButton);
    const Position AlgaeHome    = Position("AlgaeHome", ElevatorConstants::kHeightOffset, 75.0_deg,  0.0, false, ControlsConstants::kAlgaeHomeButton);
}

/// @brief Clamps the input to a specifed range
/// @param val Value to clamp
/// @param low Lower bound
/// @param high Higher bound
/// @retval val if within range
/// @retval low if below range
/// @retval high if above range
template <typename T>
static T clamp(T val, T low, T high) 
{
    return val > low && val < high ? val : val <= low ? low : high; 
}

/// @brief Returns the sign of the input
/// @param val Input to determine sign of
/// @retval 0 if val == 0
/// @retval -1 if val < 0
/// @retval 1 if val > 0
template <typename T>
inline static T sgn(T val)
{
    return val == T{0} ? T{0} : val > T{0} ? T{1} : T{-1};
}

/// @brief Returns the value at the index
/// @param inData Array
/// @param position Index
/// @retval Default value of the type if position is not within bounds of the array
/// @retval Value at position
template <typename T>
inline static T ExtractArrayEntry(const std::vector<T>& inData, int position) 
{
    if (inData.size() < static_cast<size_t>(position + 1)) {
        return T{};
    }
    return inData[position];
}