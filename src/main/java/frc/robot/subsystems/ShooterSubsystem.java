package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.controls.Follower;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

public class ShooterSubsystem extends SubsystemBase {
    private static SparkFlex centerShooterMotor = new SparkFlex(Constants.ShooterConstants.CENTER_SHOOTER_MOTOR_ID,
            MotorType.kBrushless);

    private static SparkFlex leftShooterMotor = new SparkFlex(Constants.ShooterConstants.LEFT_SHOOTER_MOTOR_ID,
            MotorType.kBrushless);

    private static SparkFlex rightShooterMotor = new SparkFlex(Constants.ShooterConstants.RIGHT_SHOOTER_MOTOR_ID,
            MotorType.kBrushless);

    private static SparkFlex indexerMotor = new SparkFlex(Constants.ShooterConstants.INDEXER_MOTOR_ID,
            MotorType.kBrushless);

    private static SparkMax leftActuatorMotor = new SparkMax(Constants.ShooterConstants.LEFT_ACTUATOR_ID,
            MotorType.kBrushless);
    private static SparkMax rightActuatorMotor = new SparkMax(Constants.ShooterConstants.RIGHT_ACTUATOR_ID,
            MotorType.kBrushless);

    private static AnalogPotentiometer leftPotentiometer = new AnalogPotentiometer(0, 1, 0);
    private static AnalogPotentiometer rightPotentiometer = new AnalogPotentiometer(0, 1, 0);

    private static double currentPotentiometerPosition; // might need second value for the right potentiometer

    public void startShooterMotors() {
        centerShooterMotor.set(Constants.ShooterConstants.SHOOTER_VELOCITY);
        leftShooterMotor.set(Constants.ShooterConstants.SHOOTER_VELOCITY);
        rightShooterMotor.set(Constants.ShooterConstants.SHOOTER_VELOCITY);
    }

    public double getShooterMotorVelocity() {
        return leftShooterMotor.getEncoder().getVelocity();
    }

    public void startIndexerMotor() {
        // if (LimelightHelpers.getTX("limelight") < 1.5 &&
        // LimelightHelpers.getTX("limelight") > -1.5) {
        indexerMotor.set(Constants.ShooterConstants.INDEXER_MOTOR_SPEED);
        // } else
        // indexerMotor.set(0);
    }

    public Command shootFuelCommand() {
        return runOnce(() -> startShooterMotors())
                .until(() -> {
                    return (getShooterMotorVelocity() >= Constants.ShooterConstants.SHOOTER_VELOCITY);
                })
                .andThen(() -> startIndexerMotor());
    }

    /*
     * public Command shootFuelCommand() {
     * return runOnce(() -> startShooterMotors()).andThen(new WaitCommand(2))
     * .andThen(() -> startIndexerMotor());
     * };
     */

    public void stopShooter() {
        centerShooterMotor.set(0);
        indexerMotor.set(0);
    }

    public Command stopShooterCommand() {
        return runOnce(() -> stopShooter());
    }

    public void moveActuator(double desiredPotentiometerPosition) {
        if (desiredPotentiometerPosition > currentPotentiometerPosition) {
            //TODO: Test for positive or negative power
            leftActuatorMotor.set(0.1);
            rightActuatorMotor.set(0.1);
        } else {
            leftActuatorMotor.set(-0.1);
            rightActuatorMotor.set(-0.1);
        }
    }

    public void stopActuator() {
        leftActuatorMotor.set(0);
        rightActuatorMotor.set(0);
    }

    public Command moveActuatorCommand(double desiredPotentiometerPosition) {
        return run(() -> moveActuator(desiredPotentiometerPosition))
                .until(() -> currentPotentiometerPosition == currentPotentiometerPosition)
                .andThen(() -> stopActuator());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Potentiometer Distance", leftPotentiometer.get());
        SmartDashboard.putNumber("Right Potentiometer Distance", rightPotentiometer.get());
        currentPotentiometerPosition = leftPotentiometer.get();
    }
}
