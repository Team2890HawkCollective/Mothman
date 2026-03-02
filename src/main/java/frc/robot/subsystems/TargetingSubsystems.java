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
import frc.robot.LimelightHelpers;
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

    PIDController photonAimPIDController = new PIDController(3, 0, 0.001);

    static Pose2d allianceHubPose;
    public static Rotation2d hubThetaPose = new Rotation2d();
    public static Optional<Alliance> alliance = DriverStation.getAlliance();

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

    public Command aimAtHubPose(SwerveSubsystem swerveDrive, CommandXboxController driverXbox) {
        return new RunCommand(() -> {
            currentRobotPose = swerveDrive.getPose();



            // Transform2d errorFromDesiredPose = desiredPose.minus(currentRobotPose);

            
            Rotation2d angleDifference = PhotonUtils.getYawToPose(currentRobotPose,
                    allianceHubPose);

            double angleSpeed = photonAimPIDController.calculate(currentRobotPose.getRotation().getRadians(),
                    angleDifference.getRadians());

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

            swerveDrive.drive(new Translation2d(driverXbox.getLeftX() * -1,
                    driverXbox.getLeftY() * -1), rot, false);
        }, swerveDrive);
    }


    public static void getHubPoseTheta(SwerveSubsystem swerveDrive)
    {
        if(alliance.isPresent()){
            if (alliance.get() == Alliance.Blue){
            hubThetaPose = new Rotation2d(Math.atan2(Constants.TargetingConstants.HUB_Y_POSE_BLUE - swerveDrive.getPose().getY(), Constants.TargetingConstants.HUB_X_POSE_BLUE - swerveDrive.getPose().getX()));

                allianceHubPose = new Pose2d(Constants.TargetingConstants.HUB_X_POSE_BLUE, Constants.TargetingConstants.HUB_Y_POSE_BLUE, hubThetaPose);
            }

            else{
                hubThetaPose = new Rotation2d(Math.atan2(Constants.TargetingConstants.HUB_Y_POSE_RED - swerveDrive.getPose().getY(), Constants.TargetingConstants.HUB_X_POSE_RED - swerveDrive.getPose().getX()));
                allianceHubPose = new Pose2d(Constants.TargetingConstants.HUB_X_POSE_RED, Constants.TargetingConstants.HUB_Y_POSE_RED, hubThetaPose);
            }
        } 

    }
   
    @Override
    public void periodic() {
        
        SmartDashboard.putString("Target Hub Pose", allianceHubPose.getX() + "\n" + allianceHubPose.getY() + "\n" + allianceHubPose.getRotation()) ;
        
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
