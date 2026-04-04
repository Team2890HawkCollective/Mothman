// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.lang.annotation.Target;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.DoubleSupplier;

import javax.lang.model.util.ElementScanner14;

import frc.robot.subsystems.TargetingSubsystems;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.swervedrive.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

        // Replace with CommandPS4Controller or CommandJoystick if needed
        final CommandXboxController driverXbox = new CommandXboxController(0);
        final CommandXboxController operatorXbox = new CommandXboxController(1);
        private final static CommandJoystick topButtons = new CommandJoystick(2);
        final CommandJoystick bottomButtons = new CommandJoystick(3);

        // The robot's subsystems and commands are defined here...
        private static final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                        "swerve/neo"));

        // Establish a Sendable Chooser that will be able to be sent to the
        // SmartDashboard, allowing selection of desired auto
        private final SendableChooser<Command> autoChooser;

        private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
        private final TargetingSubsystems m_TargetingSubsystems = new TargetingSubsystems();
        private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
        private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
        private final LEDSubsystem m_LedSubsystem = new LEDSubsystem();
        /**
         * Converts driver input into a field-relative ChassisSpeeds that is controlled
         * by angular velocity.
         */
        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> driverXbox.getLeftY() * -1,
                        () -> driverXbox.getLeftX() * -1)
                        .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
                        .deadband(OperatorConstants.DEADBAND)
                        .scaleTranslation(0.8)
                        .allianceRelativeControl(true);

        /**
         * Clone's the angular velocity input stream and converts it to a fieldRelative
         * input stream.
         */
        SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                        .withControllerHeadingAxis(driverXbox::getRightX,
                                        driverXbox::getRightY)
                        .headingWhile(true);

        /**
         * Clone's the angular velocity input stream and converts it to a robotRelative
         * input stream.
         */
        SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(false)
                        .allianceRelativeControl(true);

        SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> -driverXbox.getLeftY(),
                        () -> -driverXbox.getLeftX())
                        .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                        2))
                        .deadband(OperatorConstants.DEADBAND)
                        .scaleTranslation(0.8)
                        .allianceRelativeControl(true);
        // Derive the heading axis with math!
        SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
                        .withControllerHeadingAxis(() -> Math.sin(
                                        driverXbox.getRawAxis(
                                                        2) *
                                                        Math.PI)
                                        *
                                        (Math.PI *
                                                        2),
                                        () -> Math.cos(
                                                        driverXbox.getRawAxis(
                                                                        2) *
                                                                        Math.PI)
                                                        *
                                                        (Math.PI *
                                                                        2))
                        .headingWhile(true)
                        .translationHeadingOffset(true)
                        .translationHeadingOffset(Rotation2d.fromDegrees(
                                        0));

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the trigger bindings
                configureBindings();
                DriverStation.silenceJoystickConnectionWarning(true);

                // Create the NamedCommands that will be used in PathPlanner
                NamedCommands.registerCommand("test", Commands.print("I EXIST"));
                NamedCommands.registerCommand("Shoot_Fuel_Command",
                                m_ShooterSubsystem.shootFuelCommand());
                NamedCommands.registerCommand("Startup_Shooter_Command", m_ShooterSubsystem.startupShooterMotorsRPMAutoCommand());
                NamedCommands.registerCommand("Deploy_Intake_Command", m_IntakeSubsystem.goToPositionCommand(Constants.IntakeConstants.INTAKE_DEPLOY_ENCODER_VALUE)
                                .andThen(m_IntakeSubsystem.startIntakeWheelsCommand(
                                                Constants.IntakeConstants.INTAKE_WHEELS_MOTOR_RPM_FAST)));
                NamedCommands.registerCommand("Stop_Shooter_Command",m_ShooterSubsystem.setShooterMotorsRPMIdleCommand());
                NamedCommands.registerCommand("Retract_Intake_Command", m_IntakeSubsystem.goToPositionCommand(Constants.IntakeConstants.INTAKE_RETRACT_ENCODER_VALUE));               
                NamedCommands.registerCommand("Lift_Robot_Command", m_ClimberSubsystem.liftRobotCommand());
                NamedCommands.registerCommand("Assist_Shooter",
                                m_IntakeSubsystem.assistShooterCommand());
                NamedCommands.registerCommand("Lift_Robot", m_ClimberSubsystem.liftRobotCommand());
                NamedCommands.registerCommand("Kill_All", killAllCommand());
                NamedCommands.registerCommand("Auto_Aim_To_Hub",
                                m_TargetingSubsystems.aimAtHubPoseAutonomousMode(drivebase, driverXbox).repeatedly());

                /*
                 * NamedCommands.registerCommand("PathPlan_To_Climb_Right_Offsetted",
                 * drivebase.driveToClimbPoseOffsetted(
                 * Constants.TargetingConstants.BLUE_RIGHT_CLIMB_POSE_OFFSETTED,
                 * Constants.TargetingConstants.RED_RIGHT_CLIMB_POSE_OFFSETTED));
                 * NamedCommands.registerCommand("PathPlan_To_Climb_Left_Offsetted",
                 * drivebase.driveToClimbPoseOffsetted(
                 * Constants.TargetingConstants.BLUE_LEFT_CLIMB_POSE_OFFSETTED,
                 * Constants.TargetingConstants.RED_LEFT_CLIMB_POSE_OFFSETTED));
                 * NamedCommands.registerCommand("PathPlan_Into_Climb_Right",
                 * drivebase.driveToClimbPoseOffsetted(Constants.TargetingConstants.
                 * BLUE_RIGHT_CLIMB_POSE,
                 * Constants.TargetingConstants.RED_RIGHT_CLIMB_POSE));
                 * NamedCommands.registerCommand("PathPlan_Into_Climb_Left",
                 * drivebase.driveToClimbPoseOffsetted(Constants.TargetingConstants.
                 * BLUE_LEFT_CLIMB_POSE,
                 * Constants.TargetingConstants.RED_LEFT_CLIMB_POSE));
                 */

                // Have the autoChooser pull in all PathPlanner autos as options
                autoChooser = AutoBuilder.buildAutoChooser();

                // Set the default auto (do nothing)
                autoChooser.setDefaultOption("Do Nothing", Commands.none());

                // Add a simple auto option to have the robot drive forward for 1 second then
                // stop

                // Put the autoChooser on the SmartDashboard
                SmartDashboard.putData("Auto Chooser", autoChooser);

        }

        /**
         * Use this method to define your trigger->command mappings. Triggers can be
         * created via the
         * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
         * an arbitrary predicate, or via the
         * named factories in
         * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
         * for
         * {@link CommandXboxController
         * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
         * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
         * Flight joysticks}.
         */
        private void configureBindings() {
                Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
                Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
                Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
                Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
                                driveDirectAngle);
                Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
                Command driveFieldOrientedAnglularVelocityKeyboard = drivebase
                                .driveFieldOriented(driveAngularVelocityKeyboard);
                Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
                                driveDirectAngleKeyboard);

                driverXbox.start().onTrue(m_IntakeSubsystem.stopIntakeWheelsCommand());
                driverXbox.back().onTrue((Commands.runOnce(drivebase::zeroGyro)));

                driverXbox.leftTrigger().onTrue(m_IntakeSubsystem.startIntakeWheelsCommand(Constants.IntakeConstants.INTAKE_WHEELS_MOTOR_RPM_FAST));
                driverXbox.rightBumper()
                                .whileTrue(m_IntakeSubsystem.reverseIntakeWheelsCommand()
                                                .andThen(m_ShooterSubsystem.reverseIndexerAndRampMotorRPMCommand()))
                                                //.andThen(m_ShooterSubsystem.reverseShooterCommand()))
                                .onFalse(m_IntakeSubsystem.startIntakeWheelsCommand(Constants.IntakeConstants.INTAKE_WHEELS_MOTOR_RPM_FAST)
                                                .andThen(m_ShooterSubsystem.stopIndexerAndRampMotorCommand()));
                                                //.andThen(m_ShooterSubsystem.setShooterMotorsRPMIdleCommand()));
                // command for
                // full shooting system including linear actuators
                // driverXbox.rightTrigger().onTrue(m_ShooterSubsystem.shootFuelCommand().andThen(new
                // WaitCommand(1.5)));
                // .andThen(m_IntakeSubsystem.assistFuelIntakeCommand().repeatedly()));
                driverXbox.rightTrigger().onTrue(m_ShooterSubsystem.shootFuelCommand().andThen(m_IntakeSubsystem.assistShooterCommand().andThen(m_IntakeSubsystem.startIntakeWheelsCommand(Constants.IntakeConstants.INTAKE_WHEELS_MOTOR_RPM_SLOW))));
                driverXbox.leftBumper().onTrue(m_IntakeSubsystem
                                .startIntakeWheelsCommand(Constants.IntakeConstants.INTAKE_WHEELS_MOTOR_RPM_SLOW));
                // driverXbox.rightBumper().onTrue(m_IntakeSubsystem.assistFuelIntakeCommand(Constants.IntakeConstants.INTAKE_THROUGHBORE_ENCODER_MIDDLE,
                // Constants.IntakeConstants.INTAKE_THROUGHBORE_ENCODER_DEPLOY).repeatedly());

                driverXbox.y().onTrue(m_ClimberSubsystem.lowerRobotCommand());
                driverXbox.a().onTrue(m_ClimberSubsystem.liftRobotCommand());
                // driverXbox.povDown().onTrue(m_IntakeSubsystem.goToPositionCommand(Constants.IntakeConstants.INTAKE_THROUGHBORE_ENCODER_DEPLOY));
                // driverXbox.povUp().onTrue(m_IntakeSubsystem.goToPositionCommand(Constants.IntakeConstants.INTAKE_THROUGHBORE_ENCODER_RETRACT));
                // driverXbox.povLeft().onTrue(m_IntakeSubsystem.goToPositionCommand(Constants.IntakeConstants.INTAKE_THROUGHBORE_ENCODER_MIDDLE));

                driverXbox.povDown().onTrue(m_IntakeSubsystem.goToPositionCommand(Constants.IntakeConstants.INTAKE_DEPLOY_ENCODER_VALUE));
                driverXbox.povUp().onTrue(m_IntakeSubsystem.goToPositionCommand(Constants.IntakeConstants.INTAKE_RETRACT_ENCODER_VALUE));
                // driverXbox.povRight().whileTrue(m_TargetingSubsystems.aimAtHubPose(drivebase,
                // driverXbox));

                // driverXbox.rightTrigger().onTrue(m_ShooterSubsystem.shootFuelCommand());
                driverXbox.x().onTrue(m_ShooterSubsystem.setShooterMotorsRPMIdleCommand()
                        .andThen(m_IntakeSubsystem.goToPositionCommand(Constants.IntakeConstants.INTAKE_DEPLOY_ENCODER_VALUE))
                        .andThen((m_IntakeSubsystem.startIntakeWheelsCommand(Constants.IntakeConstants.INTAKE_WHEELS_MOTOR_RPM_FAST))));

                driverXbox.b().whileTrue(drivebase.lockSwerveCommand().andThen(m_LedSubsystem.setLEDLockedStatusCommand(true))).onFalse(m_LedSubsystem.setLEDLockedStatusCommand(false));
                // .andThen(m_IntakeSubsystem.goToPositionCommand(Constants.IntakeConstants.INTAKE_THROUGHBORE_ENCODER_DEPLOY)));
                // driverXbox.a().whileTrue(aimAtHopperCommand(() -> -driverXbox.getLeftY(),
                // () -> -driverXbox.getLeftX()));

                // driverXbox.b().whileTrue(m_TargetingSubsystems.aimAndRangeToPose(Constants.TargetingConstants.LEFT_CLIMB_POSE));

                bottomButtons.button(12).whileTrue(m_ShooterSubsystem.testLeftShooterCommand())
                                .onFalse(m_ShooterSubsystem.stopLeftShooterCommand());
                bottomButtons.button(11).whileTrue(m_ShooterSubsystem.testCenterShooterCommand())
                                .onFalse(m_ShooterSubsystem.stopCenterShooterCommand());
                bottomButtons.button(10).whileTrue(m_ShooterSubsystem.testRightShooterCommand())
                                .onFalse(m_ShooterSubsystem.stopRightShooterCommand());

                topButtons.axisGreaterThan(1, 0.3)
                                .toggleOnTrue(m_IntakeSubsystem.rotateIntakeManualCommand(
                                                Constants.IntakeConstants.INTAKE_MANUAL_SPEED * 3))
                                .toggleOnFalse(m_IntakeSubsystem.setUseArmRotationAutomaticStatusCommand(true));
                topButtons.axisGreaterThan(1, -0.3)
                                .toggleOnTrue(m_IntakeSubsystem
                                                .rotateIntakeManualCommand(Constants.IntakeConstants.INTAKE_MANUAL_SPEED *-1))
                                .toggleOnFalse(m_IntakeSubsystem.setUseArmRotationAutomaticStatusCommand(true));
                topButtons.axisGreaterThan(0, 0.5).onTrue(m_ClimberSubsystem.setClimberSpeedCommand(0.4))
                                .onFalse(m_ClimberSubsystem.setClimberSpeedCommand(0));
                topButtons.axisLessThan(0, -0.8).onTrue(m_ClimberSubsystem.setClimberSpeedCommand(-0.4))
                                .onFalse(m_ClimberSubsystem.setClimberSpeedCommand(0));
                topButtons.button(3).onTrue(killAllCommand());
                topButtons.button(6).whileTrue(m_TargetingSubsystems.aimAtHubPose(drivebase, driverXbox));
                topButtons.button(1)
                                .onTrue(drivebase.driveToClimbPoseOffsetted(
                                                Constants.TargetingConstants.BLUE_LEFT_CLIMB_POSE_OFFSETTED,
                                                Constants.TargetingConstants.RED_LEFT_CLIMB_POSE_OFFSETTED));
                topButtons.button(2)
                                .onTrue(drivebase.driveToClimbPoseOffsetted(
                                                Constants.TargetingConstants.BLUE_RIGHT_CLIMB_POSE_OFFSETTED,
                                                Constants.TargetingConstants.RED_RIGHT_CLIMB_POSE_OFFSETTED));
                topButtons.button(4).whileTrue(m_ShooterSubsystem.manualIndexerCommand())
                                .onFalse(m_ShooterSubsystem.stopIndexerAndRampMotorCommand());

                bottomButtons.button(9)
                                .onTrue(m_IntakeSubsystem.assistShooterCommand());
                //bottomButtons.button(3).whileTrue(m_IntakeSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
                //bottomButtons.button(7).whileTrue(m_IntakeSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
                //bottomButtons.button(4).whileTrue(m_IntakeSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
                //bottomButtons.button(8).whileTrue(m_IntakeSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

                //operatorXbox.povUp().onTrue(m_IntakeSubsystem.goToPositionCommand(Constants.IntakeConstants.INTAKE_RETRACT_ENCODER_VALUE));
                //operatorXbox.povDown().onTrue(m_IntakeSubsystem.goToPositionCommand(Constants.IntakeConstants.INTAKE_COLLECT_ENCODER_VALUE));
                //operatorXbox.x().whileTrue(m_IntakeSubsystem.assistShooterCommand().repeatedly());
                //.assistFuelIntakeCommand(Constants.IntakeConstants.INTAKE_COLLECT_ENCODER_VALUE, Constants.IntakeConstants.INTAKE_MIDDLE_ENCODER_VALUE).repeatedly()).onFalse(m_IntakeSubsystem.goToPositionCommand(Constants.IntakeConstants.INTAKE_COLLECT_ENCODER_VALUE));


                // topButtons.button(1).onTrue(drivebase.driveToPose(Constants.))

                if (RobotBase.isSimulation()) {
                        drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
                } else {
                        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
                }

                if (Robot.isSimulation()) {
                        Pose2d target = new Pose2d(new Translation2d(1, 4),
                                        Rotation2d.fromDegrees(90));
                        // drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
                        driveDirectAngleKeyboard.driveToPose(() -> target,
                                        new ProfiledPIDController(5,
                                                        0,
                                                        0,
                                                        new Constraints(5, 2)),
                                        new ProfiledPIDController(5,
                                                        0,
                                                        0,
                                                        new Constraints(Units.degreesToRadians(360),
                                                                        Units.degreesToRadians(180))));
                        driverXbox.start()
                                        .onTrue(Commands.runOnce(() -> drivebase
                                                        .resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
                        driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
                        driverXbox.button(2)
                                        .whileTrue(Commands.runEnd(
                                                        () -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                                                        () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

                        // driverXbox.b().whileTrue(
                        // drivebase.driveToPose(
                        // new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                        // );

                }
                if (DriverStation.isTest()) {
                        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command
                                                                                         // above!

                        driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
                        driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
                        driverXbox.back().whileTrue(drivebase.centerModulesCommand());
                        driverXbox.leftBumper().onTrue(Commands.none());
                        driverXbox.rightBumper().onTrue(Commands.none());
                } else {
                        // driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
                        // driverXbox.start().whileTrue(Commands.none());
                        // driverXbox.back().whileTrue(Commands.none());
                        // driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock,
                        // drivebase).repeatedly());
                        // driverXbox.rightBumper().onTrue(Commands.none());
                }

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // Pass in the selected auto from the SmartDashboard as our desired autnomous
                // commmand
                return autoChooser.getSelected();
        }

        public void setMotorBrake(boolean brake) {
                drivebase.setMotorBrake(brake);
        }

        public SwerveSubsystem getSwerveDrive() {
                return drivebase;
        }

        public void killAll() {
                CommandScheduler.getInstance().cancelAll();
        }

        public Command killAllCommand() {
                return Commands.runOnce(() -> killAll());
        }



}