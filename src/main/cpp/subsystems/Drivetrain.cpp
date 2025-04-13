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
            pathApplyRobotSpeeds.WithSpeeds(speeds)
                .WithWheelForceFeedforwardsX(feedforwards.robotRelativeForcesX)
                .WithWheelForceFeedforwardsY(feedforwards.robotRelativeForcesY);
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