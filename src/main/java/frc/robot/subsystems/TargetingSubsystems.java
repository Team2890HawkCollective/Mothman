package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants;

public class TargetingSubsystems extends SubsystemBase {

    PIDController photonAimPIDController = new PIDController(3, 0, 0.001);

    public TargetingSubsystems() {
        photonAimPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    Pose2d currentRobotPose;

    public List<Waypoint> pathWaypoints;

    public Command pathPlanToPoseCommand(Pose2d desiredPose, SwerveSubsystem swerveDrive) {
        GoalEndState goalEndState = new GoalEndState(0, desiredPose.getRotation());
        PathConstraints pathConstraints = new PathConstraints(3.0, 3.0, 3.0, 6.0, 12.0);
        currentRobotPose = swerveDrive.getPose();
        pathWaypoints = PathPlannerPath.waypointsFromPoses(
                currentRobotPose, desiredPose);

        PathPlannerPath goToDesiredPose = new PathPlannerPath(pathWaypoints, pathConstraints, null,
                goalEndState);
        goToDesiredPose.preventFlipping = true;

        return swerveDrive.getAutonomousCommand("goToDesiredPose");
    }

    public Command aimAndRangeToPose(Pose2d desiredPose, SwerveSubsystem swerveDrive) {
        return new RunCommand(() -> {
            currentRobotPose = swerveDrive.getPose();

            Transform2d errorFromDesiredPose = desiredPose.minus(currentRobotPose);

            double xError = errorFromDesiredPose.getX();
            double yError = errorFromDesiredPose.getY();
            double angleError = errorFromDesiredPose.getRotation().getRadians();

            PIDController xController = new PIDController(1.5, 0, 0);
            PIDController yController = new PIDController(1.5, 0, 0);

            double xSpeed = xController.calculate(currentRobotPose.getX(), desiredPose.getX());
            double ySpeed = yController.calculate(currentRobotPose.getY(), desiredPose.getY());
            double angleSpeed = photonAimPIDController.calculate(currentRobotPose.getRotation().getRadians(),
                    desiredPose.getRotation().getRadians());

            swerveDrive.drive(new Translation2d(xSpeed, ySpeed), angleSpeed, false);
        }, swerveDrive);
    }

    public Command aimAtHub(SwerveSubsystem swerveDrive, CommandXboxController driverXbox) {
        return new RunCommand(() -> {
            currentRobotPose = swerveDrive.getPose();

            // Transform2d errorFromDesiredPose = desiredPose.minus(currentRobotPose);

            Rotation2d angleDifference = PhotonUtils.getYawToPose(currentRobotPose,
                    Constants.TargetingConstants.HUB_POSE);

            double angleSpeed = photonAimPIDController.calculate(currentRobotPose.getRotation().getRadians(),
                    angleDifference.getRadians());

            angleSpeed = MathUtil.clamp(angleSpeed, -3.0, 3.0);

            swerveDrive.drive(new Translation2d(driverXbox.getLeftX() * -1, -driverXbox.getLeftY() * -1), angleSpeed,
                    false);
        }, swerveDrive);
    }

    Command photonAimAtClimb(SwerveSubsystem swerveDrive, CommandXboxController driverXbox) {
        return new RunCommand(() -> {
            double rot = 0.0;
            var result = Constants.TargetingConstants.RED_PHOTON_CAM.getLatestResult();
            if (result.hasTargets()) {
                double yawError = result.getBestTarget().getYaw();
                rot = photonAimPIDController.calculate(yawError, 0);
            }

            rot = MathUtil.clamp(rot, -3.0, 3.0);

            swerveDrive.drive(new Translation2d(driverXbox.getLeftX() * -1,
                    driverXbox.getLeftY() * -1), rot, false);
        }, swerveDrive);
    }

    public PhotonPoseEstimator getPhotonPoseEstimator(PhotonPoseEstimator poseEstimator) {
        return poseEstimator;
    }

    /*
     * // static public NetworkTable table =
     * //
     * NetworkTableInstance.getDefault().getTable(Constants.LimeLight.LIMELIGHT_NAME
     * );
     * // static public NetworkTableEntry ty = table.getEntry("ty");
     * // static double targetOffsetAngle_Vertical = ty.getDouble(0.0);
     * 
     * // how many degrees back is your limelight rotated from perfectly vertical?
     * static double limelightMountAngleDegrees = 25.0;
     * 
     * // distance from the center of the Limelight lens to the floor
     * static double limelightLensHeightInches = 27.5;
     * 
     * // distance from the target to the floor
     * static double goalHeightInches = 44;
     * 
     * static double angleToGoalDegrees = limelightMountAngleDegrees +
     * Constants.LimeLight.LIMELIGHT_TY;
     * static double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
     * 
     * // calculate distance
     * static double distanceFromLimelightToGoalInches = (goalHeightInches -
     * limelightLensHeightInches)
     * / Math.tan(angleToGoalRadians);
     * 
     * public static double getDistanceFromAprilTag() {
     * angleToGoalDegrees = limelightMountAngleDegrees +
     * Constants.LimeLight.LIMELIGHT_TY;
     * angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
     * distanceFromLimelightToGoalInches = (goalHeightInches -
     * limelightLensHeightInches)
     * / Math.tan(angleToGoalRadians);
     * return distanceFromLimelightToGoalInches;
     * }
     */

    public static void updateRobotPose(PhotonCamera camera, PhotonPoseEstimator poseEstimator,
            SwerveSubsystem swerveDrive) {
        Optional<EstimatedRobotPose> result = poseEstimator.update(camera.getLatestResult());

        if (result.isPresent()) {
            EstimatedRobotPose estimatedPose = result.get();
            swerveDrive.getSwerveDrive()
                    .addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
        }
    }

    @Override
    public void periodic() {
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
    }
}
