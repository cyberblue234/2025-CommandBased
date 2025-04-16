#include "subsystems/Drivetrain.h"

Drivetrain::Drivetrain() : swerve::SwerveDrivetrain<hardware::TalonFX, hardware::TalonFX, hardware::CANcoder>(drivetrainConstants, frontLeft, frontRight, backLeft, backRight)
{    
    // Configure the AutoBuilder last
    AutoBuilder::configure(
        [this](){ return GetState().Pose; }, // Robot pose supplier
        [this](frc::Pose2d pose){ ResetPose(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this](){ return GetState().Speeds; }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](frc::ChassisSpeeds speeds, DriveFeedforwards feedforwards)
        { 
            SetControl(pathApplyRobotSpeeds.WithSpeeds(speeds)
                .WithWheelForceFeedforwardsX(feedforwards.robotRelativeForcesX)
                .WithWheelForceFeedforwardsY(feedforwards.robotRelativeForcesY));
        }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        std::make_shared<PPHolonomicDriveController>(translationPIDs, rotationPIDs),
        PathPlannerConstants::kConfig, // The robot configuration
        []() {
            // Boolean supplier that controls when the path will be mirrored for the blue alliance
            // This will flip the path being followed to the blue side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == frc::DriverStation::Alliance::kBlue;
            }
            return false;
        },
        this // Reference to this subsystem to set requirements
    );

    frc::SmartDashboard::PutData("Field", &field);
}

void Drivetrain::Periodic()
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

    swerve::impl::SwerveDrivetrainImpl::SwerveDriveState state = GetState();
    field.SetRobotPose(state.Pose);
    odometryPublisher.Set(state.Pose);
    speedsPublisher.Set(state.Speeds);
    modulePositionsPublisher.Set(state.ModulePositions);
    moduleStatesPublisher.Set(state.ModuleStates);
    moduleTargetsPublisher.Set(state.ModuleTargets);
}

frc2::CommandPtr Drivetrain::DriveWithSpeedsCommand(std::function<frc::ChassisSpeeds()> speedsSupplier)
{
    return Run([this, speedsSupplier]
    {
        frc::ChassisSpeeds speeds = speedsSupplier();
        SetControl(drive.WithVelocityX(speeds.vx).WithVelocityY(speeds.vy).WithRotationalRate(speeds.omega));
    }).WithName("DriveWithSpeeds");
}