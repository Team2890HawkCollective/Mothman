// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
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
    public static final double MAX_SPEED_MPS = Units.feetToMeters(14.5);
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

        public static double SHOOTER_RPM;

        public static void updateShooterRPM() {
            SHOOTER_RPM = shooterRPM.getDouble(-4000);
        }

        public static final int CENTER_SHOOTER_MOTOR_ID = 42;
        public static final int LEFT_SHOOTER_MOTOR_ID = 41;
        public static final int RIGHT_SHOOTER_MOTOR_ID = 40;
        public static final int INDEXER_MOTOR_ID = 43;
        
        public static final int SHOOTER_MOTOR_CURRENT_LIMIT = 80;

        public static final double SHOOTER_MOTOR_P = 0.0018;
        public static final double SHOOTER_MOTOR_I = 0;
        public static final double SHOOTER_MOTOR_D = 0;

        public static final double INDEXER_MOTOR_P = 0.0001;
        public static final double INDEXER_MOTOR_I = 0;
        public static final double INDEXER_MOTOR_D = 0;

        public static final double CENTER_MOTOR_S = 0.0;
        public static final double CENTER_MOTOR_V = 0.0;

        public static final double LEFT_MOTOR_S = 0.0;
        public static final double LEFT_MOTOR_V = 0.0;

        public static final double RIGHT_MOTOR_S = 0.0;
        public static final double RIGHT_MOTOR_V = 0.0;


        /*private static GenericEntry indexerAndRampRPM = programmingTab.add("Desired Ramp + Indexer RPM", 2000)
                .withWidget(BuiltInWidgets.kNumberBar).getEntry();*/


        public static double INDEXER_AND_RAMP_MOTOR_RPM = 10000;

        // this method called in robot periodic so values updated in elastic are
        // constantly read and applied to RAMP_MOTOR_SPEED
        /*public static void updateIndexerAndRampMotorRPM() {
            INDEXER_AND_RAMP_MOTOR_RPM = indexerAndRampRPM.getDouble(2000);
        }*/
    }

    public static class IntakeConstants {

        /* private static GenericEntry intakeRPM = programmingTab.add("Desired Intake RPM", -1000)
                .withWidget(BuiltInWidgets.kNumberBar).getEntry(); */
        public static double INTAKE_WHEELS_MOTOR_RPM = -7000;

        /*public static void updateIntakeWheelsRPM() {
            INTAKE_WHEELS_MOTOR_RPM = intakeRPM.getDouble(-1000);
        }*/

        public static final int INTAKE_WHEELS_MOTOR_ID = 50;
        public static final int INTAKE_WHEELS_CURRENT_LIMIT = 60;
        public static final double INTAKE_WHEELS_POSITION_CONVERSION_FACTOR = 2 * Math.PI; // Encoder Unit * Conversion Factor = Radians. 1 Rotation = 2PI Radians
        public static final double INTAKE_WHEELS_VELOCITY_CONVERSION_FACTOR = INTAKE_WHEELS_POSITION_CONVERSION_FACTOR * 60.0; // Encoder Units per Minute * Conversion Factor = Radians per Second

        public static final int INTAKE_ROTATOR_MOTOR_ID = 51;
        public static final int INTAKE_ROTATOR_CURRENT_LIMIT = 40;
        public static final double INTAKE_ROTATOR_INITIAL_ENCODER_VALUE = Units.degreesToRadians(-90);
        public static final double INTAKE_ROTATOR_POSITION_CONVERSION_FACTOR = (2 * Math.PI) / 12.0; // Encoder Unit * Conversion Factor = Radians. 1 Rotation = 2PI Radians. Gear ratio is 12:1, so 12 Rotations = 1 full intake Rotation.
        public static final double INTAKE_ROTATOR_VELOCITY_CONVERSION_FACTOR = INTAKE_ROTATOR_POSITION_CONVERSION_FACTOR * 60.0; // Encoder Units per Minute * Conversion Factor = Radians per Second

        public static final double INTAKE_WHEELS_MOTOR_P = 0.0001; // Radians -> Radians Per Second
        public static final double INTAKE_WHEELS_MOTOR_I = 0.0; // Radians -> Radians Per Second
        public static final double INTAKE_WHEELS_MOTOR_D = 0.0; // Radians -> Radians Per Second
        public static final double INTAKE_WHEELS_MOTOR_S = 0.0; // Voltage to overcome static friction/inertia
        public static final double INTAKE_WHEELS_MOTOR_V = 0.0; // Voltage per Rads/Second gain

        public static final double INTAKE_ROTATOR_DOWN_P = 0.03; // Radians -> Radians Per Second
        public static final double INTAKE_ROTATOR_DOWN_I = 0.0; // Radians -> Radians Per Second
        public static final double INTAKE_ROTATOR_DOWN_D = 0.0; // Radians -> Radians Per Second

        public static final double INTAKE_ROTATOR_UP_P = 0.06; // Radians -> Radians Per Second
        public static final double INTAKE_ROTATOR_UP_I = 0.0; // Radians -> Radians Per Second
        public static final double INTAKE_ROTATOR_UP_D = 0.0; // Radians -> Radians Per Second

        public static final double INTAKE_ROTATOR_MOTOR_S = 0.0; // Voltage to overcome static friction/inertia. V
        public static final double INTAKE_ROTATOR_MOTOR_G = 0.0; // Voltage to overcome Gravity. V
        public static final double INTAKE_ROTATOR_MOTOR_V = 0.0; // Voltage per Radians/Second. V/(Rad/s)

        public static final double INTAKE_COLLECT_POSITION_RADS = 0.0; 
        public static final double INTAKE_MIDDLE_POSITION_RADS = -Units.degreesToRadians(45);
        public static final double INTAKE_RETRACT_POSITION_RADS = -Units.degreesToRadians(90);
    }


        // create object and a new widget under programming tab in Elastic where object
        // retrieves value from widget



    public static class TargetingConstants {
        public static final Pose2d RIGHT_CLIMB_POSE_METERS = new Pose2d(1.075, 4.75, Rotation2d.fromDegrees(90));
        public static final Pose2d LEFT_CLIMB_POSE_METERS = new Pose2d(1.075, 2.75, Rotation2d.fromDegrees(-90));

        public static final Vector<N3> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4, 4, 8);
        public static final Vector<N3> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);

        public static final Pose2d HUB_POSE_METERS = new Pose2d(4.625, 4.03, new Rotation2d());

        public static final Translation3d FRONT_LEFT_CAMERA_LOCATION_METERS = new Translation3d(0, 0, 0);
        public static final Translation3d FRONT_RIGHT_CAMERA_LOCATION_METERS = new Translation3d(0, 0, 0);
        public static final Translation3d REAR_LEFT_CAMERA_LOCATION_METERS = new Translation3d(0, 0, 0);
        public static final Translation3d REAR_RIGHT_CAMERA_LOCATION_METERS = new Translation3d(0, 0, 0);

        public static final Rotation3d FRONT_LEFT_CAMERA_ANGLE_RADIANS = new Rotation3d(0, 0, 0);
        public static final Rotation3d FRONT_RIGHT_CAMERA_ANGLE_RADIANS = new Rotation3d(0, 0, 0);
        public static final Rotation3d REAR_LEFT_CAMERA_ANGLE_RADIANS = new Rotation3d(0, 0, 0);
        public static final Rotation3d REAR_RIGHT_CAMERA_ANGLE_RADIANS = new Rotation3d(0, 0, 0);
    }

    public static class ClimberConstants {
        public static final int CLIMB_MOTOR_ID = 60;
        public static final int RATCHET_PWM_PORT = 9;

        public static final double RATCHET_UNLOCK_ANGLE_DEGREES = 0;
        public static final double RATCHET_LOCK_ANGLE_DEGREES = 180;
        public static final double CLIMBER_SPEED_DUTY_CYCLE = .5;
    }
}
