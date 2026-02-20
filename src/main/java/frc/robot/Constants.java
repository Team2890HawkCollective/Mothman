// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

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
    public static final double MAX_SPEED = Units.feetToMeters(14.5);
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
        private static GenericEntry shooterVelocity = programmingTab.add("Desired Shooter RPM", -1000)
                .withWidget(BuiltInWidgets.kNumberBar).getEntry();

        public static double SHOOTER_RPM;

        public static void getShooterVelocity() {
            SHOOTER_RPM = shooterVelocity.getDouble(-1000);
        }

        public static final int CENTER_SHOOTER_MOTOR_ID = 42;
        public static final int LEFT_SHOOTER_MOTOR_ID = 41;
        public static final int RIGHT_SHOOTER_MOTOR_ID = 40;
        public static final int INDEXER_MOTOR_ID = 43;
        public static double INDEXER_AND_RAMP_MOTOR_RPM;

        public static double SHOOTER_MOTOR_P = 0.001;
        public static double SHOOTER_MOTOR_I = 0;
        public static double SHOOTER_MOTOR_D = 0;

        private static GenericEntry indexerAndRampRPM = programmingTab.add("Desired Ramp + Indexer Speed", 1000)
                .withWidget(BuiltInWidgets.kNumberBar).getEntry();

        // this method called in robot periodic so values updated in elastic are
        // constantly read and applied to RAMP_MOTOR_SPEED
        public static void getIndexerAndRampMotorRPM() {
            INDEXER_AND_RAMP_MOTOR_RPM = indexerAndRampRPM.getDouble(1000);
        }
    }

    public static class IntakeConstants {
        private static GenericEntry intakeSpeed = programmingTab.add("Desired Intake Speed", -0.65)
                .withWidget(BuiltInWidgets.kNumberBar).getEntry();

        public static double INTAKE_WHEELS_MOTOR_SPEED;

        private static GenericEntry intakeRPM = programmingTab.add("Desired Intake RPM", -1000)
                .withWidget(BuiltInWidgets.kNumberBar).getEntry();
        public static double INTAKE_WHEELS_MOTOR_RPM;

        public static void getIntakeWheelsSpeed() {
            INTAKE_WHEELS_MOTOR_RPM = intakeRPM.getDouble(-1000);
        }

        public static final int INTAKE_WHEELS_MOTOR_ID = 50;
        public static final int INTAKE_ROTATOR_MOTOR_ID = 51;

        public static class IntakeRotatorPID {
            public static final double INTAKE_ROTATOR_P = 0.01;
            public static final double INTAKE_ROTATOR_I = 0;
            public static final double INTAKE_ROTATOR_D = 0;
        }

        public static final double INTAKE_COLLECT_ENCODER_VALUE = 4.1290459632873535;
        public static final double INTAKE_MIDDLE_ENCODER_VALUE = 1.2550222873687744;
        public static final double INTAKE_RETRACT_ENCODER_VALUE = 0;
    }

    public static class RampConstants {
        public static final int RAMP_MOTOR_ID = 45;
        // create object and a new widget under programming tab in Elastic where object
        // retrieves value from widget
        private static GenericEntry rampSpeed = programmingTab.add("Desired Ramp Speed", 0.4)
                .withWidget(BuiltInWidgets.kNumberBar).getEntry();

        // this method called in robot periodic so values updated in elastic are
        // constantly read and applied to RAMP_MOTOR_SPEED
        public static void getRampMotorSpeed() {
            RAMP_MOTOR_SPEED = rampSpeed.getDouble(0.4);
        }

        public static double RAMP_MOTOR_SPEED = .6;

        // create object and a new widget under programming tab in Elastic where object
        // retrieves value from widget

    }

    public static class TargetingConstants {
        public static final Pose2d RIGHT_CLIMB_POSE = new Pose2d(1.075, 4.75, Rotation2d.fromDegrees(90));
        public static final Pose2d LEFT_CLIMB_POSE = new Pose2d(1.075, 2.75, Rotation2d.fromDegrees(-90));

        public static final PhotonCamera ORANGE_PHOTON_CAM = new PhotonCamera("Arducam_OV9281_USB_Camera");
        public static final PhotonCamera BLACK_PHOTON_CAM = new PhotonCamera("Arducam_OV9281_USB_Camera");
        public static final PhotonCamera RED_PHOTON_CAM = new PhotonCamera("Arducam_OV9281_USB_Camera");
        public static final PhotonCamera PURPLE_PHOTON_CAM = new PhotonCamera("Arducam_OV9281_USB_Camera");

        public static final Transform3d ORANGE_ROBOT_TO_CAM = new Transform3d(new Translation3d(0, 0, 0),
                new Rotation3d(0, 0, 0));
        public static final Transform3d BLACK_ROBOT_TO_CAM = new Transform3d(new Translation3d(0, 0, 0),
                new Rotation3d(0, 0, 0));
        public static final Transform3d RED_ROBOT_TO_CAM = new Transform3d(new Translation3d(0, 0, 0),
                new Rotation3d(0, 0, 0));
        public static final Transform3d PURPLE_ROBOT_TO_CAM = new Transform3d(new Translation3d(0, 0, 0),
                new Rotation3d(0, 0, 0));

        public static final PhotonPoseEstimator ORANGE_PHOTON_ESTIMATOR = new PhotonPoseEstimator(
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark),
                ORANGE_ROBOT_TO_CAM);
        public static final PhotonPoseEstimator BLACK_PHOTON_ESTIMATOR = new PhotonPoseEstimator(
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark),
                BLACK_ROBOT_TO_CAM);
        public static final PhotonPoseEstimator RED_PHOTON_ESTIMATOR = new PhotonPoseEstimator(
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark),
                RED_ROBOT_TO_CAM);
        public static final PhotonPoseEstimator PURPLE_PHOTON_ESTIMATOR = new PhotonPoseEstimator(
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark),
                PURPLE_ROBOT_TO_CAM);
    }

    public static class ClimberConstants {
        public static final int CLIMB_MOTOR_ID = 60;
        public static final int RATCHET_PWM_PORT = 9;

        public static final double RATCHET_UNLOCK_ANGLE = 0;
        public static final double RATCHET_LOCK_ANGLE = 180;
        public static final double CLIMBER_SPEED = .5;
    }
}
