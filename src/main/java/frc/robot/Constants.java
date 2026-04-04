// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    private static ShuffleboardTab programmingTab = Shuffleboard.getTab("Programming");

    public static final double ROBOT_MASS = 115 * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
    public static final double MAX_SPEED = Units.feetToMeters(19.3);
    // Maximum speed of the robot in meters per second, used to limit acceleration.

    // public static final class AutonConstants
    // {
    //
    // public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0,
    // 0);
    // public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
    // }

    public static final class DrivebaseConstants {

        // Hold time on motor brakes when disabled
        public static final double WHEEL_LOCK_TIME = 10; // seconds
    }

    public static class OperatorConstants {

        // Joystick Deadband
        public static final double DEADBAND = 0.1;
        public static final double LEFT_Y_DEADBAND = 0.1;
        public static final double RIGHT_X_DEADBAND = 0.1;
        public static final double TURN_CONSTANT = 6;
    }

    public static class ShooterConstants {
        private static GenericEntry shooterRPM = programmingTab.add("Desired Shooter RPM", -4000)
                .withWidget(BuiltInWidgets.kNumberBar).getEntry();

        public static double SHOOTER_RPM = -2700;
        public static final double IDLE_SHOOTER_RPM = -1800;
        public static final double AUTO_SHOOTER_RPM = -3000; //always negative to shoot

        public static void updateShooterRPM() {
            SHOOTER_RPM = shooterRPM.getDouble(-4000);
        }


        public static double SHOOTER_RPM_LEFT;
        public static double SHOOTER_RPM_RIGHT;
        public static double SHOOTER_RPM_CENTER;
        public static final int CENTER_SHOOTER_MOTOR_ID = 42;
        public static final int LEFT_SHOOTER_MOTOR_ID = 41;
        public static final int RIGHT_SHOOTER_MOTOR_ID = 40;
        public static final int INDEXER_MOTOR_ID = 43;

        public static double SHOOTER_MOTOR_P = 0.001;
        public static double SHOOTER_MOTOR_I = 0.0;
        public static double SHOOTER_MOTOR_D = 0.001;
        public static double SHOOTER_MOTOR_S = 0.2;
        public static double SHOOTER_MOTOR_V = 0.0015;

        public static double INDEXER_MOTOR_P = 0.0001;
        public static double INDEXER_MOTOR_I = 0;
        public static double INDEXER_MOTOR_D = 0;

        /*
         * private static GenericEntry indexerAndRampRPM =
         * programmingTab.add("Desired Ramp + Indexer RPM", 2000)
         * .withWidget(BuiltInWidgets.kNumberBar).getEntry();
         */

        public static double INDEXER_AND_RAMP_MOTOR_RPM = 15000; //always positive when feeding into shooter

        // this method called in robot periodic so values updated in elastic are
        // constantly read and applied to RAMP_MOTOR_SPEED
        /*
         * public static void updateIndexerAndRampMotorRPM() {
         * INDEXER_AND_RAMP_MOTOR_RPM = indexerAndRampRPM.getDouble(2000);
         * }
         */
    }

    public static class IntakeConstants {

        /*
         * private static GenericEntry intakeRPM =
         * programmingTab.add("Desired Intake RPM", -1000)
         * .withWidget(BuiltInWidgets.kNumberBar).getEntry();
         
         
         
        \*/
        public static double INTAKE_WHEELS_MOTOR_RPM_FAST = -5000; // always negative when intaking
        public static double INTAKE_WHEELS_MOTOR_RPM_SLOW = -3000; // always negative when intaking


        /*
         * public static void updateIntakeWheelsRPM() {
         * INTAKE_WHEELS_MOTOR_RPM = intakeRPM.getDouble(-1000);
         * }
         */

        public static final int INTAKE_WHEELS_MOTOR_ID = 50;
        public static final int INTAKE_ROTATOR_MOTOR_ID = 51;

        public static class IntakeRotatorPID {
            public static final double INTAKE_ROTATOR_P = .07;
            public static final double INTAKE_ROTATOR_I = 0;
            public static final double INTAKE_ROTATOR_D = 0.03;
        }

        public static final double INTAKE_WHEELS_P = 0.0001;
        public static final double INTAKE_WHEELS_I = 0.000001;
        public static final double INTAKE_WHEELS_D = 0;

        public static final double INTAKE_DEPLOY_ENCODER_VALUE = 0;
        public static final double INTAKE_ASSIST_ENCODER_VALUE = 4.8;
        public static final double INTAKE_RETRACT_ENCODER_VALUE = 6.2;

        public static final double INTAKE_THROUGHBORE_ENCODER_DEPLOY = .13;
        public static final double INTAKE_THROUGHBORE_ENCODER_RETRACT = .49;
        public static final double INTAKE_THROUGHBORE_ENCODER_MIDDLE = .389;

        public static final double INTAKE_MANUAL_SPEED = .15;
    }

    // create object and a new widget under programming tab in Elastic where object
    // retrieves value from widget

    public static class TargetingConstants {
        public static final Pose2d BLUE_LEFT_CLIMB_POSE = new Pose2d(1.14, 4.25, Rotation2d.fromDegrees(90));
        public static final Pose2d BLUE_LEFT_CLIMB_POSE_OFFSETTED = new Pose2d(1.20, 5.0,
                Rotation2d.fromDegrees(90));

        public static final Pose2d BLUE_RIGHT_CLIMB_POSE = new Pose2d(1.14, 2.046, Rotation2d.fromDegrees(-90));
        public static final Pose2d BLUE_RIGHT_CLIMB_POSE_OFFSETTED = new Pose2d(1.20, 3.25,
                Rotation2d.fromDegrees(-90));

        public static final Pose2d RED_RIGHT_CLIMB_POSE = new Pose2d(15.35, 5, Rotation2d.fromDegrees(90));
        public static final Pose2d RED_RIGHT_CLIMB_POSE_OFFSETTED = new Pose2d(15.34, 5.72,
                Rotation2d.fromDegrees(90));

        public static final Pose2d RED_LEFT_CLIMB_POSE = new Pose2d(15.35, 3.6, Rotation2d.fromDegrees(-90));
        public static final Pose2d RED_LEFT_CLIMB_POSE_OFFSETTED = new Pose2d(15.34, 3.1,
                Rotation2d.fromDegrees(-90));

        public static final PhotonCamera ORANGE_PHOTON_CAM = new PhotonCamera("Back Left Camera");
        public static final PhotonCamera BLACK_PHOTON_CAM = new PhotonCamera("Back Right Camera");
        public static final PhotonCamera RED_PHOTON_CAM = new PhotonCamera("Front Left Camera");
        public static final PhotonCamera PURPLE_PHOTON_CAM = new PhotonCamera("Front Right Camera");

        public static Rotation2d HUB_ROTATION_BLUE;
        public static Rotation2d HUB_ROTATION_RED;
        public static Pose2d allianceHubPose = new Pose2d();


        public static final double HUB_X_POSE_BLUE = 4.625;
        public static final double HUB_Y_POSE_BLUE = 4.03;

        public static final double HUB_X_POSE_RED = 11.915;
        public static final double HUB_Y_POSE_RED = 4.03;

        public static PathConstraints DRIVE_INTO_CLIMB_CONSTRAINTS = new PathConstraints(1.5, 3.0, 3.0, 6.0, 12.0);

        public static final Transform3d ROBOT_TO_BACK_LEFT_CAM = new Transform3d(
                new Translation3d(-Units.inchesToMeters(12.75), Units.inchesToMeters(6.25),
                        Units.inchesToMeters(13.1875)),
                new Rotation3d(0, 0, Math.toRadians(200)));

        public static final Transform3d ROBOT_TO_BACK_RIGHT_CAM = new Transform3d(
                new Translation3d(-Units.inchesToMeters(12.75), -Units.inchesToMeters(6.25), Units.inchesToMeters(13.1875)),
                new Rotation3d(0, 0, Math.toRadians(-200)));

        public static final Transform3d ROBOT_TO_FRONT_LEFT_CAM = new Transform3d(
                new Translation3d(Units.inchesToMeters(1.3), Units.inchesToMeters(11.25), Units.inchesToMeters(27)),
                new Rotation3d(0, Math.toRadians(10), Math.toRadians(10)));

        public static final Transform3d ROBOT_TO_FRONT_RIGHT_CAM = new Transform3d(
                new Translation3d(Units.inchesToMeters(1.3), Units.inchesToMeters(-11.25), Units.inchesToMeters(27)),
                new Rotation3d(0, Math.toRadians(10), Math.toRadians(-10)));
    }

    public static class ClimberConstants {
        public static final int CLIMB_MOTOR_ID = 52;

        public static final double CLIMBER_SPEED = .5;

        public static final double CLIMBER_PID_P = 5;
        public static final double CLIMBER_PID_I = 0;
        public static final double CLIMBER_PID_D = 0;

        public static final double CLIMBER_LIFTED_SETPOINT_VALUE = 65;
        public static final double CLIMBER_LOWERED_SETPOINT_VALUE = 0;

    }

    public static class LEDConstants {
        public static final int LED_PWM_PORT = 5;

    }
}