package frc.robot.subsystems;

import java.lang.StackWalker.Option;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.dyn4j.geometry.Rotation;
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
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

    PIDController photonAimPIDController = new PIDController(5, 0.01, 0);

    public static Rotation2d hubThetaPose = new Rotation2d();
    public static Optional<Alliance> alliance = DriverStation.getAlliance();
    private static ShuffleboardTab cameras;

    public TargetingSubsystems() {
        photonAimPIDController.enableContinuousInput(-Math.PI, Math.PI);
        cameras = Shuffleboard.getTab("Vision");
        // cameras.addCamera("Rear Left Camera","Rear Left
        // Camera","http://photonvision.local:5800/#/cameras");
        // cameras.addCamera("Rear Right Camera", "Rear Right Camera",
        // "http://photonvision.local:5800/#/cameras");

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

    public Command aimAtHubPose(SwerveSubsystem swerveDrive, CommandXboxController driverXbox) {
        return new RunCommand(() -> {
            currentRobotPose = swerveDrive.getPose();

            // Transform2d errorFromDesiredPose = desiredPose.minus(currentRobotPose);

            Rotation2d angleDifference = PhotonUtils.getYawToPose(currentRobotPose,
                    Constants.TargetingConstants.allianceHubPose);

            double angleSpeed = photonAimPIDController.calculate(currentRobotPose.getRotation().getRadians(), Constants.TargetingConstants.allianceHubPose.getRotation().getRadians());

            angleSpeed = MathUtil.clamp(angleSpeed, -3.0, 3.0);

            swerveDrive.drive(new Translation2d(driverXbox.getLeftX() * -1, -driverXbox.getLeftY() * -1), angleSpeed,
                    true);
        }, swerveDrive);
    }

    Command photonAimAtAprilTag(SwerveSubsystem swerveDrive, CommandXboxController driverXbox) {
        return new RunCommand(() -> {
            double rot = 0.0;
            var result = Constants.TargetingConstants.RED_PHOTON_CAM.getLatestResult();
            if (result.hasTargets()) {
                double yawError = result.getBestTarget().getYaw();
                rot = photonAimPIDController.calculate(yawError, 0);
            }

            rot = MathUtil.clamp(rot, -3.0, 3.0);

            swerveDrive.drive(new Translation2d(driverXbox.getLeftY() * -1,
                    driverXbox.getLeftX() * -1), rot, false);
        }, swerveDrive);
    }

    public static void getHubPoseTheta(SwerveSubsystem swerveDrive) {
        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Blue) {
                hubThetaPose = new Rotation2d(
                        Math.atan2(Constants.TargetingConstants.HUB_Y_POSE_BLUE - swerveDrive.getPose().getY(), Constants.TargetingConstants.HUB_X_POSE_BLUE - swerveDrive.getPose().getX()));

                Constants.TargetingConstants.allianceHubPose = new Pose2d(Constants.TargetingConstants.HUB_X_POSE_BLUE,
                        Constants.TargetingConstants.HUB_Y_POSE_BLUE, hubThetaPose);
            }

            else {
                hubThetaPose = new Rotation2d(
                        Math.atan2(Constants.TargetingConstants.HUB_Y_POSE_RED - swerveDrive.getPose().getY(),
                        Constants.TargetingConstants.HUB_X_POSE_RED - swerveDrive.getPose().getX()));
                Constants.TargetingConstants.allianceHubPose = new Pose2d(Constants.TargetingConstants.HUB_X_POSE_RED,
                        Constants.TargetingConstants.HUB_Y_POSE_RED, hubThetaPose);
            }
        }

    }


      public static void updateShooterRPM(Pose2d currentRobotPose) {
      double distance = PhotonUtils.getDistanceToPose(currentRobotPose,
      Constants.TargetingConstants.allianceHubPose);
      Constants.ShooterConstants.SHOOTER_RPM = Math.max((-293.84123 * Math.pow(distance, 3))
      + (1360.01497 * Math.pow(distance, 2))
      - (2391.17127 * distance)
      - 1249.22704, -6000); 
     }
     

    @Override
    public void periodic() {
        alliance = DriverStation.getAlliance();
        SmartDashboard.putString("Target Hub Pose",
                Constants.TargetingConstants.allianceHubPose.getX() + " " + Constants.TargetingConstants.allianceHubPose.getY() + " " + Constants.TargetingConstants.allianceHubPose.getRotation());

        SmartDashboard.putString("Hub Pose", "x: " + Constants.TargetingConstants.allianceHubPose.getMeasureX() + "  y: " + Constants.TargetingConstants.allianceHubPose.getY()
                + "  angle: " + Constants.TargetingConstants.allianceHubPose.getRotation());
    }
}
