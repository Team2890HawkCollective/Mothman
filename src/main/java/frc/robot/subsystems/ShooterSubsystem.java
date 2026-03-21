package frc.robot.subsystems;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ShooterSubsystem extends SubsystemBase {

    public static boolean useIndexerAutomaticStatus = false;
    public static boolean enableShooter = false;

    private static SparkFlex centerShooterMotor = new SparkFlex(Constants.ShooterConstants.CENTER_SHOOTER_MOTOR_ID,
            MotorType.kBrushless);

    private static SparkFlex leftShooterMotor = new SparkFlex(Constants.ShooterConstants.LEFT_SHOOTER_MOTOR_ID,
            MotorType.kBrushless);

    private static SparkFlex rightShooterMotor = new SparkFlex(Constants.ShooterConstants.RIGHT_SHOOTER_MOTOR_ID,
            MotorType.kBrushless);

    private static SparkFlex indexerAndRampMotor = new SparkFlex(Constants.ShooterConstants.INDEXER_MOTOR_ID,
            MotorType.kBrushless);

    private static SparkClosedLoopController centerShooterMotorPIDController;
    public static SparkFlexConfig centerShooterMotorConfig = new SparkFlexConfig();

    private static SparkClosedLoopController leftShooterMotorPIDController;
    public static SparkFlexConfig leftShooterMotorConfig = new SparkFlexConfig();

    private static SparkClosedLoopController rightShooterMotorPIDController;
    public static SparkFlexConfig rightShooterMotorConfig = new SparkFlexConfig();

    private static SparkClosedLoopController indexerAndRampMotorPIDController;
    public static SparkFlexConfig indexerAndRampMotorConfig = new SparkFlexConfig();

    private static BangBangController bangBangController = new BangBangController();

    public ShooterSubsystem() {

        useIndexerAutomaticStatus = false;
        enableShooter = false;

        centerShooterMotorConfig
                .voltageCompensation(12.0).closedLoop
                .outputRange(-1, 0)
                .p(Constants.ShooterConstants.SHOOTER_MOTOR_P)
                .i(Constants.ShooterConstants.SHOOTER_MOTOR_I)
                .d(Constants.ShooterConstants.SHOOTER_MOTOR_D)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder).feedForward
                .kS(Constants.ShooterConstants.SHOOTER_MOTOR_S)
                .kV(Constants.ShooterConstants.SHOOTER_MOTOR_V);
        centerShooterMotorConfig.smartCurrentLimit(60);

        centerShooterMotor.configure(centerShooterMotorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters,
                com.revrobotics.PersistMode.kNoPersistParameters);
        centerShooterMotorPIDController = centerShooterMotor.getClosedLoopController();

        leftShooterMotorConfig
                .voltageCompensation(12.0).closedLoop
                .outputRange(-1, 0)
                .p(Constants.ShooterConstants.SHOOTER_MOTOR_P)
                .i(Constants.ShooterConstants.SHOOTER_MOTOR_I)
                .d(Constants.ShooterConstants.SHOOTER_MOTOR_D)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder).feedForward
                .kS(Constants.ShooterConstants.SHOOTER_MOTOR_S)
                .kV(Constants.ShooterConstants.SHOOTER_MOTOR_V);
        leftShooterMotorConfig.smartCurrentLimit(60);
        leftShooterMotor.configure(leftShooterMotorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters,
                com.revrobotics.PersistMode.kNoPersistParameters);
        leftShooterMotorPIDController = leftShooterMotor.getClosedLoopController();

        rightShooterMotorConfig
                .voltageCompensation(12.0).closedLoop
                .outputRange(-1, 0)
                .p(Constants.ShooterConstants.SHOOTER_MOTOR_P)
                .i(Constants.ShooterConstants.SHOOTER_MOTOR_I)
                .d(Constants.ShooterConstants.SHOOTER_MOTOR_D)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder).feedForward
                .kS(Constants.ShooterConstants.SHOOTER_MOTOR_S)
                .kV(Constants.ShooterConstants.SHOOTER_MOTOR_V);
        rightShooterMotorConfig.smartCurrentLimit(60);
        rightShooterMotor.configure(rightShooterMotorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters,
                com.revrobotics.PersistMode.kNoPersistParameters);
        rightShooterMotorPIDController = rightShooterMotor.getClosedLoopController();

        indexerAndRampMotorConfig.closedLoop.pid(Constants.ShooterConstants.INDEXER_MOTOR_P,
                0,
                0);
        indexerAndRampMotorConfig.smartCurrentLimit(60);
        indexerAndRampMotor.configure(indexerAndRampMotorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters,
                com.revrobotics.PersistMode.kNoPersistParameters);
        indexerAndRampMotorPIDController = indexerAndRampMotor.getClosedLoopController();
    }

    public void setShooterMotorsRPM() {
        enableShooter = true;

        // centerShooterMotorPIDController.setSetpoint(Constants.ShooterConstants.SHOOTER_RPM_CENTER,
        // ControlType.kVelocity);
        // leftShooterMotorPIDController.setSetpoint(Constants.ShooterConstants.SHOOTER_RPM_LEFT,
        // ControlType.kVelocity);
        // rightShooterMotorPIDController.setSetpoint(Constants.ShooterConstants.SHOOTER_RPM_RIGHT,
        // ControlType.kVelocity);
    }

    public static void startupShooterMotorsRPMAuto() {
        enableShooter = true;
        // centerShooterMotorPIDController.setSetpoint(Constants.ShooterConstants.SHOOTER_RPM_CENTER,
        // ControlType.kVelocity);
        // leftShooterMotorPIDController.setSetpoint(Constants.ShooterConstants.SHOOTER_RPM_LEFT,
        // ControlType.kVelocity);
        // rightShooterMotorPIDController.setSetpoint(Constants.ShooterConstants.SHOOTER_RPM_RIGHT,
        // ControlType.kVelocity);
    }

    public static void setShooterMotorsRPMIdle() {
        enableShooter = false;
        useIndexerAutomaticStatus = false;
        centerShooterMotorPIDController.setSetpoint(Constants.ShooterConstants.IDLE_SHOOTER_RPM, ControlType.kVelocity);
        leftShooterMotorPIDController.setSetpoint(Constants.ShooterConstants.IDLE_SHOOTER_RPM, ControlType.kVelocity);
        rightShooterMotorPIDController.setSetpoint(Constants.ShooterConstants.IDLE_SHOOTER_RPM, ControlType.kVelocity);
    }

    public Command setShooterMotorsRPMIdleCommand() {
        return runOnce(() -> setShooterMotorsRPMIdle())
                .andThen(stopIndexerAndRampMotorCommand());
    }

    public Command startupShooterMotorsRPMAutoCommand() {
        return runOnce(() -> startupShooterMotorsRPMAuto());
    }

    public Command setShooterMotorsRPMAutoCommand() {
        return runOnce(() -> setShooterMotorsRPM());
    }

    // test individual motor code
    public void setLeftShooterMotorRPM() {
        leftShooterMotorPIDController.setSetpoint(Constants.ShooterConstants.SHOOTER_RPM, ControlType.kVelocity);
    }

    public Command testLeftShooterCommand() {
        return runOnce(() -> setLeftShooterMotorRPM());
    }

    public void setRightShooterMotorRPM() {
        rightShooterMotorPIDController.setSetpoint(Constants.ShooterConstants.SHOOTER_RPM, ControlType.kVelocity);
    }

    public Command testRightShooterCommand() {
        return runOnce(() -> setRightShooterMotorRPM());
    }

    public void setCenterShooterMotorRPM() {
        centerShooterMotorPIDController.setSetpoint(Constants.ShooterConstants.SHOOTER_RPM, ControlType.kVelocity);
    }

    public Command testCenterShooterCommand() {
        return runOnce(() -> setCenterShooterMotorRPM());
    }

    public void stopLeftShooterMotorRPM() {
        leftShooterMotor.set(0);
    }

    public Command stopLeftShooterCommand() {
        return runOnce(() -> stopLeftShooterMotorRPM());
    }

    public void stopCenterShooterMotorRPM() {
        centerShooterMotor.set(0);
    }

    public Command stopCenterShooterCommand() {
        return runOnce(() -> stopCenterShooterMotorRPM());
    }

    public void stopRightShooterMotorRPM() {
        rightShooterMotor.set(0);
    }

    public Command stopRightShooterCommand() {
        return runOnce(() -> stopRightShooterMotorRPM());
    }

    public double getShooterMotorRPM() {
        return rightShooterMotor.getEncoder().getVelocity();
    }

    public void setIndexerAndRampMotorRPM() {

        if (useIndexerAutomaticStatus == true) {

            if (leftShooterMotor.getEncoder().getVelocity() <= Constants.ShooterConstants.SHOOTER_RPM_LEFT * .70
                    && centerShooterMotor.getEncoder().getVelocity() <= Constants.ShooterConstants.SHOOTER_RPM_CENTER
                            * .70
                    && rightShooterMotor.getEncoder().getVelocity() <= Constants.ShooterConstants.SHOOTER_RPM_RIGHT
                            * .70) {
                indexerAndRampMotorPIDController.setSetpoint(Constants.ShooterConstants.INDEXER_AND_RAMP_MOTOR_RPM,
                        ControlType.kVelocity);
            } else
                stopIndexerAndRampMotor();

        }
    }

    public void manualIndexer() {
        useIndexerAutomaticStatus = false;
        indexerAndRampMotorPIDController.setSetpoint(Constants.ShooterConstants.INDEXER_AND_RAMP_MOTOR_RPM,
                ControlType.kVelocity);
    }

    public Command manualIndexerCommand() {
        return runOnce(() -> manualIndexer());
    }

    public void reverseIndexerAndRampMotorRPM() {
        useIndexerAutomaticStatus = false;
        indexerAndRampMotorPIDController.setSetpoint(Constants.ShooterConstants.INDEXER_AND_RAMP_MOTOR_RPM * -1,
                ControlType.kVelocity);
    }

    public Command reverseIndexerAndRampMotorRPMCommand() {
        return runOnce(() -> reverseIndexerAndRampMotorRPM());
    }

    public Command setIndexerAndRampMotorRPMCommand() {
        return runOnce(() -> setIndexerAndRampMotorRPM());
    }

    public Command stopIndexerAndRampMotorCommand() {
        return runOnce(() -> stopIndexerAndRampMotor());
    }

    public void stopIndexerAndRampMotor() {
        useIndexerAutomaticStatus = false;
        indexerAndRampMotor.set(0);
    }

    public void setIndexerStatusTrue() {
        useIndexerAutomaticStatus = true;
    }


    /*
     * public Command shootFuelCommand() {
     * return runOnce(() -> setShooterMotorsRPM())
     * .until(() -> {
     * return (getShooterMotorRPM() <= Constants.ShooterConstants.SHOOTER_RPM);
     * })
     * .andThen(() -> setIndexerAndRampMotorRPM());
     * }
     */

    /*
     * public Command shootFuelCommand() {
     * return runOnce(() -> setShooterMotorsRPM()).andThen(new WaitCommand(1.5))
     * .andThen(() -> setIndexerAndRampMotorRPM());
     * };
     */

    public Command shootFuelCommand() {
        return runOnce(() -> setShooterMotorsRPM()).until(
                () -> leftShooterMotor.getEncoder().getVelocity() <= Constants.ShooterConstants.SHOOTER_RPM_LEFT * .95
                        && centerShooterMotor.getEncoder().getVelocity() <= Constants.ShooterConstants.SHOOTER_RPM_CENTER * .95
                        && rightShooterMotor.getEncoder().getVelocity() <= Constants.ShooterConstants.SHOOTER_RPM_RIGHT * .95)
                .andThen(() -> setIndexerStatusTrue());
    };

    public void stopShooters() {

        enableShooter = false;
        useIndexerAutomaticStatus = false;
        centerShooterMotor.set(0);
        leftShooterMotor.set(0);
        rightShooterMotor.set(0);
        indexerAndRampMotor.set(0);

    }

    public Command stopShooterCommand() {
        return runOnce(() -> stopShooters());
    }

    public void reverseShooter() {
        centerShooterMotor.set(.4);
        leftShooterMotor.set(0.4);
        rightShooterMotor.set(0.4);
    }

    public Command reverseShooterCommand() {
        return runOnce(() -> reverseShooter());
    }

    @Override
    public void periodic() {

        if (enableShooter == true) {
            if (leftShooterMotor.getEncoder().getVelocity() >= Constants.ShooterConstants.SHOOTER_RPM_LEFT * .95)
                leftShooterMotor.set(bangBangController.calculate(leftShooterMotor.getEncoder().getVelocity() * -1,
                        Constants.ShooterConstants.SHOOTER_RPM_LEFT * -1) * -1);
            else
                leftShooterMotorPIDController.setSetpoint(Constants.ShooterConstants.SHOOTER_RPM_LEFT,
                        ControlType.kVelocity);

            if (centerShooterMotor.getEncoder().getVelocity() >= Constants.ShooterConstants.SHOOTER_RPM_CENTER * .95)
                centerShooterMotor.set(bangBangController.calculate(centerShooterMotor.getEncoder().getVelocity() * -1,
                        Constants.ShooterConstants.SHOOTER_RPM_CENTER * -1) * -1);
            else
                centerShooterMotorPIDController.setSetpoint(Constants.ShooterConstants.SHOOTER_RPM_CENTER,
                        ControlType.kVelocity);

            if (rightShooterMotor.getEncoder().getVelocity() >= Constants.ShooterConstants.SHOOTER_RPM_RIGHT * .95)
                rightShooterMotor.set(bangBangController.calculate(rightShooterMotor.getEncoder().getVelocity() * -1,
                        Constants.ShooterConstants.SHOOTER_RPM_RIGHT * -1) * -1);
            else
                rightShooterMotorPIDController.setSetpoint(Constants.ShooterConstants.SHOOTER_RPM_RIGHT,
                        ControlType.kVelocity);

        }
        setIndexerAndRampMotorRPM();

        SmartDashboard.putString("Shooter Velocity", "Left: "
                + leftShooterMotor.getEncoder().getVelocity()
                + "  Center: " + centerShooterMotor.getEncoder().getVelocity()
                + "  Right: " + rightShooterMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooters RPMS/Left Shooter RPM", Constants.ShooterConstants.SHOOTER_RPM_LEFT);
        SmartDashboard.putNumber("Shooters RPMS/Right Shooter RPM", Constants.ShooterConstants.SHOOTER_RPM_RIGHT);
        SmartDashboard.putNumber("Shooters RPMS/Center Shooter RPM", Constants.ShooterConstants.SHOOTER_RPM_CENTER);

        SmartDashboard.putNumber("Indexer Velocity", indexerAndRampMotor.getEncoder().getVelocity());

    }
}
