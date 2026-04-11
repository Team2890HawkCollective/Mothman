package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;


import org.photonvision.PhotonUtils;


import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import com.pathplanner.lib.path.Waypoint;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;


public class TargetingSubsystems extends SubsystemBase {

    PIDController photonAimPIDController = new PIDController(10, 0.01, 0);

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
                    Constants.TargetingConstants.allianceHubPose);

            double angleSpeed = photonAimPIDController.calculate(currentRobotPose.getRotation().getRadians(),
                    Constants.TargetingConstants.allianceHubPose.getRotation().getRadians());

            angleSpeed = MathUtil.clamp(angleSpeed, -3.0, 3.0);

            swerveDrive.drive(new Translation2d(driverXbox.getLeftY()/2, driverXbox.getLeftX()/2), angleSpeed,
                    true);
        }, swerveDrive);
    }

    public Command aimAtHubPoseAutonomousMode(SwerveSubsystem swerveDrive, CommandXboxController driverXbox) {
        return new RunCommand(() -> {
            currentRobotPose = swerveDrive.getPose();

            // Transform2d errorFromDesiredPose = desiredPose.minus(currentRobotPose);


            double angleSpeed = photonAimPIDController.calculate(currentRobotPose.getRotation().getRadians(),
                    Constants.TargetingConstants.allianceHubPose.getRotation().getRadians());

            angleSpeed = MathUtil.clamp(angleSpeed, -3.0, 3.0);

            swerveDrive.drive(new Translation2d(), angleSpeed,
                    true);
        }).until(() -> Math.abs(hubThetaPose.getRadians() - swerveDrive.getPose().getRotation().getRadians()) <= .2).andThen(() -> swerveDrive.drive(new ChassisSpeeds(0,0,0)));
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
                        Math.atan2(
                                (Constants.TargetingConstants.HUB_Y_POSE_BLUE - swerveDrive.getFieldVelocity().vyMetersPerSecond * 1.3) - swerveDrive.getPose().getY(),
                                (Constants.TargetingConstants.HUB_X_POSE_BLUE - swerveDrive.getFieldVelocity().vxMetersPerSecond * 1.3) - swerveDrive.getPose().getX()));

                Constants.TargetingConstants.allianceHubPose = new Pose2d(Constants.TargetingConstants.HUB_X_POSE_BLUE,
                        Constants.TargetingConstants.HUB_Y_POSE_BLUE, hubThetaPose);
            }

            else {
                hubThetaPose = new Rotation2d(
                        Math.atan2((Constants.TargetingConstants.HUB_Y_POSE_RED - swerveDrive.getFieldVelocity().vyMetersPerSecond * 1.3) - swerveDrive.getPose().getY(),
                                (Constants.TargetingConstants.HUB_X_POSE_RED - swerveDrive.getFieldVelocity().vxMetersPerSecond * 1.3) - swerveDrive.getPose().getX()));
                Constants.TargetingConstants.allianceHubPose = new Pose2d(Constants.TargetingConstants.HUB_X_POSE_RED,
                        Constants.TargetingConstants.HUB_Y_POSE_RED, hubThetaPose);
            }
        }

    }

    public static void updateShooterAndIndexerRPM(Pose2d currentRobotPose) {
        double distance = PhotonUtils.getDistanceToPose(currentRobotPose,
                Constants.TargetingConstants.allianceHubPose);
        Constants.ShooterConstants.SHOOTER_RPM_LEFT = Math.max((-1.73146 * Math.pow(distance, 4))
                + (27.27766 * Math.pow(distance, 3))
                - (154.79287 * Math.pow(distance, 2))
                - (34.29619 * distance)
                - 2510.13374, -15000);

        Constants.ShooterConstants.SHOOTER_RPM_RIGHT = Math.max((-2.40765 * Math.pow(distance, 4))
                + (38.94472 * Math.pow(distance, 3))
                - (225.17963 * Math.pow(distance, 2))
                + (138.9699 * distance)
                - 2630.33326, -15000);

        Constants.ShooterConstants.SHOOTER_RPM_CENTER = Math.max((-1.84547 * Math.pow(distance, 4))
                + (32.75767 * Math.pow(distance, 3))
                - (201.29209 * Math.pow(distance, 2))
                + (58.06248 * distance)
                - 2520.16313, -15000);

        //Constants.ShooterConstants.INDEXER_AND_RAMP_MOTOR_RPM = -Constants.ShooterConstants.SHOOTER_RPM_CENTER * 4.5 * 5
         //       / 2;
        // To find the linear speed, the equation is RPM * Circumference, pi is not
        // needed as it cancels out.
    }

    @Override
    public void periodic() {
        alliance = DriverStation.getAlliance();
        SmartDashboard.putString("Target Hub Pose",
                Constants.TargetingConstants.allianceHubPose.getX() + " "
                        + Constants.TargetingConstants.allianceHubPose.getY() + " "
                        + Constants.TargetingConstants.allianceHubPose.getRotation());

        SmartDashboard.putString("Hub Pose",
                "x: " + Constants.TargetingConstants.allianceHubPose.getMeasureX() + "  y: "
                        + Constants.TargetingConstants.allianceHubPose.getY()
                        + "  angle: " + Constants.TargetingConstants.allianceHubPose.getRotation());
    }
}
