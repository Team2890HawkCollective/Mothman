package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
<<<<<<< HEAD
=======
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
>>>>>>> a50d67d7f53337b144e2b1afb5f7b644747fb21f
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants;

public class TargetingSubsystems extends SubsystemBase {
<<<<<<< HEAD

    RobotContainer m_RobotContainer = new RobotContainer();

=======
    
>>>>>>> a50d67d7f53337b144e2b1afb5f7b644747fb21f
    PhotonCamera photonVision = new PhotonCamera("Arducam_OV9281_USB_Camera");
    Transform3d BACK_LEFT_CAMERA_OFFSETS = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
    PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark),
            BACK_LEFT_CAMERA_OFFSETS);
    PIDController photonAimPIDController = new PIDController(0.3, 0, 0.001);

    public TargetingSubsystems() {
        photonAimPIDController.enableContinuousInput(-180, 180);
    }

    Pose2d currentRobotPose;

    public List<Waypoint> rightClimbWaypoints;

<<<<<<< HEAD
    public Command pathPlanToRightClimbPoseCommand() {
        GoalEndState goalEndState = new GoalEndState(0, Constants.TargetingConstants.RIGHT_CLIMB_POSE.getRotation());
        PathConstraints goToClimbConstraints = new PathConstraints(3.0, 3.0, 3.0, 6.0, 12.0);
        currentRobotPose = m_RobotContainer.getSwerveDriveBase().getPose();
=======
    public Command pathPlanToRightClimbPoseCommand(SwerveSubsystem swerveDrive) {
        GoalEndState goalEndState = new GoalEndState(0, Constants.TargetingConstants.RIGHT_CLIMB_POSE.getRotation());
        PathConstraints goToClimbConstraints = new PathConstraints(3.0, 3.0, 3.0, 6.0, 12.0);
        currentRobotPose = swerveDrive.getPose();
>>>>>>> a50d67d7f53337b144e2b1afb5f7b644747fb21f
        rightClimbWaypoints = PathPlannerPath.waypointsFromPoses(
                currentRobotPose, Constants.TargetingConstants.RIGHT_CLIMB_POSE);

        PathPlannerPath goToClimbPath = new PathPlannerPath(rightClimbWaypoints, goToClimbConstraints, null,
                goalEndState);
        goToClimbPath.preventFlipping = true;

<<<<<<< HEAD
        return m_RobotContainer.getSwerveDriveBase().getAutonomousCommand("goToClimbPath");
    }

    public Command aimAndRangeToPose(Pose2d desiredPose) {
        return new RunCommand(() -> {
         currentRobotPose = m_RobotContainer.getSwerveDriveBase().getPose();
=======
        return swerveDrive.getAutonomousCommand("goToClimbPath");
    }

    public Command aimAndRangeToPose(Pose2d desiredPose, SwerveSubsystem swerveDrive) {
        return new RunCommand(() -> {
         currentRobotPose = swerveDrive.getPose();
>>>>>>> a50d67d7f53337b144e2b1afb5f7b644747fb21f

            Transform2d errorFromDesiredPose = desiredPose.minus(currentRobotPose);

            double xError = errorFromDesiredPose.getX();
            double yError = errorFromDesiredPose.getY();
            double angleError = errorFromDesiredPose.getRotation().getRadians();

            PIDController xController = new PIDController(1.5, 0, 0);
            PIDController yController = new PIDController(1.5, 0, 0);
            PIDController angleController = new PIDController(3.0, 0, 0);

            angleController.enableContinuousInput(-Math.PI, Math.PI);

            double xSpeed = xController.calculate(currentRobotPose.getX(), desiredPose.getX());
            double ySpeed = yController.calculate(currentRobotPose.getY(), desiredPose.getY());
            double angleSpeed = angleController.calculate(currentRobotPose.getRotation().getRadians(),
                    desiredPose.getRotation().getRadians());
            
<<<<<<< HEAD
            m_RobotContainer.getSwerveDriveBase().drive(new Translation2d(xSpeed, ySpeed), angleSpeed, true);
        }, m_RobotContainer.getSwerveDriveBase());
    }

    Command photonAimAtClimb = new RunCommand(() -> {
=======
            swerveDrive.drive(new Translation2d(xSpeed, ySpeed), angleSpeed, true);
        }, swerveDrive);
    }

    Command photonAimAtClimb(SwerveSubsystem swerveDrive, CommandXboxController driverXbox) { 
        return new RunCommand(() -> {
>>>>>>> a50d67d7f53337b144e2b1afb5f7b644747fb21f
        double rot = 0.0;
        var result = photonVision.getLatestResult();
        if (result.hasTargets()) {
            double yawError = result.getBestTarget().getYaw();
            rot = photonAimPIDController.calculate(yawError, 0);
        }

        rot = MathUtil.clamp(rot, -3.0, 3.0);

<<<<<<< HEAD
        m_RobotContainer.getSwerveDriveBase().drive(new Translation2d(m_RobotContainer.getDriverXbox().getLeftY() * -1,
                m_RobotContainer.getDriverXbox().getLeftX() * -1), rot, true);
    }, m_RobotContainer.getSwerveDriveBase());
=======
        swerveDrive.drive(new Translation2d(driverXbox.getLeftY() * -1,
                driverXbox.getLeftX() * -1), rot, true);
    }, swerveDrive);
}
>>>>>>> a50d67d7f53337b144e2b1afb5f7b644747fb21f


    public PhotonPoseEstimator getPhotonPoseEstimator() {
        return photonEstimator;
    }

    // static public NetworkTable table =
    // NetworkTableInstance.getDefault().getTable(Constants.LimeLight.LIMELIGHT_NAME);
    // static public NetworkTableEntry ty = table.getEntry("ty");
    // static double targetOffsetAngle_Vertical = ty.getDouble(0.0);

    // how many degrees back is your limelight rotated from perfectly vertical?
    static double limelightMountAngleDegrees = 25.0;

    // distance from the center of the Limelight lens to the floor
    static double limelightLensHeightInches = 27.5;

    // distance from the target to the floor
    static double goalHeightInches = 44;

    static double angleToGoalDegrees = limelightMountAngleDegrees + Constants.LimeLight.LIMELIGHT_TY;
    static double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    // calculate distance
    static double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)
            / Math.tan(angleToGoalRadians);

    public static double getDistanceFromAprilTag() {
        angleToGoalDegrees = limelightMountAngleDegrees + Constants.LimeLight.LIMELIGHT_TY;
        angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
        distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)
                / Math.tan(angleToGoalRadians);
        return distanceFromLimelightToGoalInches;
    }

<<<<<<< HEAD
    @Override
    public void periodic() {

        Optional<EstimatedRobotPose> result = photonEstimator.update(photonVision.getLatestResult());

        if (result.isPresent()) {
            EstimatedRobotPose estimatedPose = result.get();
            m_RobotContainer.getSwerveDriveBase().getSwerveDrive()
                    .addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
        }
=======
    public void updateRobotPose(SwerveSubsystem swerveDrive) {
         Optional<EstimatedRobotPose> result = photonEstimator.update(photonVision.getLatestResult());

        if (result.isPresent()) {
            EstimatedRobotPose estimatedPose = result.get();
            swerveDrive.getSwerveDrive()
                    .addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
        }
    }
    @Override
    public void periodic() {

>>>>>>> a50d67d7f53337b144e2b1afb5f7b644747fb21f
        /*
         * Shuffleboard.getTab("Vision").add("Photon Vision Yaw Value",
         * photonVision.getLatestResult().getBestTarget().getYaw());
         * Shuffleboard.getTab("Vision").add("Photon Vision Pitch Value",
         * photonVision.getLatestResult().getBestTarget().getPitch());
         * Shuffleboard.getTab("Vision").add("Limelight TX Value",
         * LimelightHelpers.getTX("limelight"));
         * Shuffleboard.getTab("Vision").add("Limelight April Tag ID",
         * LimelightHelpers.getFiducialID("limelight"));
         * Shuffleboard.getTab("Vision").addCamera("Limelight", "limelight", null);
         * Shuffleboard.getTab("Vision").addCamera("Photon",
         * "Arducam_OV9281_USB_Camera",
         * "http://photonvision.local:5800");
         */
<<<<<<< HEAD
    }
}
=======
    } 
}

>>>>>>> a50d67d7f53337b144e2b1afb5f7b644747fb21f
