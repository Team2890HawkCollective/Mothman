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
import frc.robot.LimelightHelpers;

public class ShooterSubsystem extends SubsystemBase {
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

    public ShooterSubsystem() {
        centerShooterMotorConfig
                .voltageCompensation(12.0)
                .closedLoop
                .p(Constants.ShooterConstants.SHOOTER_MOTOR_P)
                .i(Constants.ShooterConstants.SHOOTER_MOTOR_I)
                .d(Constants.ShooterConstants.SHOOTER_MOTOR_D)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .feedForward
                .kS(Constants.ShooterConstants.SHOOTER_MOTOR_S)
                .kV(Constants.ShooterConstants.SHOOTER_MOTOR_V);
        centerShooterMotorConfig.smartCurrentLimit(80);
        centerShooterMotor.configure(centerShooterMotorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters,
                com.revrobotics.PersistMode.kNoPersistParameters);
        centerShooterMotorPIDController = centerShooterMotor.getClosedLoopController();

        leftShooterMotorConfig              
                .voltageCompensation(12.0)
                .closedLoop
                .p(Constants.ShooterConstants.SHOOTER_MOTOR_P)
                .i(Constants.ShooterConstants.SHOOTER_MOTOR_I)
                .d(Constants.ShooterConstants.SHOOTER_MOTOR_D)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .feedForward
                .kS(Constants.ShooterConstants.SHOOTER_MOTOR_S)
                .kV(Constants.ShooterConstants.SHOOTER_MOTOR_V);
        leftShooterMotorConfig.smartCurrentLimit(80);
        leftShooterMotor.configure(leftShooterMotorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters,
                com.revrobotics.PersistMode.kNoPersistParameters);
        leftShooterMotorPIDController = leftShooterMotor.getClosedLoopController();

        rightShooterMotorConfig
                .voltageCompensation(12.0)
                .closedLoop
                .p(Constants.ShooterConstants.SHOOTER_MOTOR_P)
                .i(Constants.ShooterConstants.SHOOTER_MOTOR_I)
                .d(Constants.ShooterConstants.SHOOTER_MOTOR_D)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .feedForward
                .kS(Constants.ShooterConstants.SHOOTER_MOTOR_S)
                .kV(Constants.ShooterConstants.SHOOTER_MOTOR_V);
        rightShooterMotorConfig.smartCurrentLimit(80);
        rightShooterMotor.configure(rightShooterMotorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters,
                com.revrobotics.PersistMode.kNoPersistParameters);
        rightShooterMotorPIDController = rightShooterMotor.getClosedLoopController();

        indexerAndRampMotorConfig.closedLoop.pid(Constants.ShooterConstants.INDEXER_MOTOR_P,
                0,
                0);
        indexerAndRampMotor.configure(indexerAndRampMotorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters,
                com.revrobotics.PersistMode.kNoPersistParameters);
        indexerAndRampMotorPIDController = indexerAndRampMotor.getClosedLoopController();
    }

    // private static SparkMax leftActuatorMotor = new
    // SparkMax(Constants.ShooterConstants.LEFT_ACTUATOR_PWM_PORT,
    // MotorType.kBrushless);

    // private static SparkMax rightActuatorMotor = new
    // SparkMax(Constants.ShooterConstants.RIGHT_ACTUATOR_PWM_PORT,
    // MotorType.kBrushless);

    public void setShooterMotorsRPM() {
        centerShooterMotorPIDController.setSetpoint(Constants.ShooterConstants.SHOOTER_RPM, ControlType.kVelocity);
        leftShooterMotorPIDController.setSetpoint(Constants.ShooterConstants.SHOOTER_RPM, ControlType.kVelocity);
        rightShooterMotorPIDController.setSetpoint(Constants.ShooterConstants.SHOOTER_RPM, ControlType.kVelocity);
    }

    public void setShooterMotorsRPMAuto(){
        centerShooterMotorPIDController.setSetpoint(-2700, ControlType.kVelocity);
        leftShooterMotorPIDController.setSetpoint(-2700, ControlType.kVelocity);
        rightShooterMotorPIDController.setSetpoint(-2700, ControlType.kVelocity);
    }

    public Command setShooterMotorsRPMAutoCommand()
    {
        return runOnce(()-> setShooterMotorsRPMAuto()).andThen(new WaitCommand(1.5))
      .andThen(() -> setIndexerAndRampMotorRPM());
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
        indexerAndRampMotorPIDController.setSetpoint(Constants.ShooterConstants.INDEXER_AND_RAMP_MOTOR_RPM,
                ControlType.kVelocity);
    }

    public void reverseIndexerAndRampMotorRPM() {
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
        return runOnce(() -> indexerAndRampMotor.set(0));
    }

    /*public Command shootFuelCommand() {
        return runOnce(() -> setShooterMotorsRPM())
                .until(() -> {
                    return (getShooterMotorRPM() <= Constants.ShooterConstants.SHOOTER_RPM);
                })
                .andThen(() -> setIndexerAndRampMotorRPM());
    }*/

    
    public Command shootFuelCommand() {
      return runOnce(() -> setShooterMotorsRPM()).andThen(new WaitCommand(1.5))
      .andThen(() -> setIndexerAndRampMotorRPM());
      };

    /*  public Command shootFuelCommand() {
      return runOnce(() -> setShooterMotorsRPM())
        .andThen(new WaitUntilCommand(() -> {
            return (getShooterMotorRPM() <= Constants.ShooterConstants.SHOOTER_RPM);
        }))
        .andThen(() -> setIndexerAndRampMotorRPM());
    };*/
     

    public void stopShooters() {

        centerShooterMotor.set(0);
        leftShooterMotor.set(0);
        rightShooterMotor.set(0);
        indexerAndRampMotor.set(0);

    }

    public Command stopShooterCommand() {
        return runOnce(() -> stopShooters());
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Shooter Velocity", leftShooterMotor.getEncoder().getVelocity());

    }
}
