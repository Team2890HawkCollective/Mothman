package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

public class IntakeSubsystem extends SubsystemBase {

    private static SparkFlex intakeWheelsMotor = new SparkFlex(Constants.IntakeConstants.INTAKE_WHEELS_MOTOR_ID, MotorType.kBrushless);

    private static SparkFlex intakeRotatorMotor = new SparkFlex(Constants.IntakeConstants.INTAKE_ROTATOR_MOTOR_ID, MotorType.kBrushless);

    public static SparkFlexConfig intakeWheelsMotorConfig = new SparkFlexConfig();
    private static BangBangController intakeWheelsMotorBBController = new BangBangController();
    private static SimpleMotorFeedforward intakeWheelsMotorFeedforward;

    public static SparkFlexConfig intakeRotatorConfig = new SparkFlexConfig();
    private static PIDController intakeRotatorMotorPIDController;
    private static ArmFeedforward intakeRotatorMotorFeedforward;

    public IntakeSubsystem() {
        intakeWheelsMotorConfig
            .smartCurrentLimit(Constants.IntakeConstants.INTAKE_WHEELS_CURRENT_LIMIT)
            .encoder
            .positionConversionFactor(Constants.IntakeConstants.INTAKE_WHEELS_POSITION_CONVERSION_FACTOR)
            .velocityConversionFactor(Constants.IntakeConstants.INTAKE_WHEELS_VELOCITY_CONVERSION_FACTOR);
        intakeWheelsMotor.configure(intakeWheelsMotorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kNoPersistParameters);

        intakeWheelsMotorFeedforward = new SimpleMotorFeedforward(Constants.IntakeConstants.INTAKE_WHEELS_MOTOR_S, Constants.IntakeConstants.INTAKE_WHEELS_MOTOR_V);
        
        intakeRotatorConfig
            .smartCurrentLimit(Constants.IntakeConstants.INTAKE_ROTATOR_CURRENT_LIMIT)
            .encoder
            .positionConversionFactor(Constants.IntakeConstants.INTAKE_ROTATOR_POSITION_CONVERSION_FACTOR)
            .velocityConversionFactor(Constants.IntakeConstants.INTAKE_ROTATOR_VELOCITY_CONVERSION_FACTOR);
        intakeRotatorMotor.configure(intakeRotatorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kNoPersistParameters);
        intakeRotatorMotor.getEncoder().setPosition(Constants.IntakeConstants.INTAKE_ROTATOR_INITIAL_ENCODER_VALUE);

        intakeRotatorMotorPIDController = new PIDController(Constants.IntakeConstants.INTAKE_ROTATOR_UP_P, Constants.IntakeConstants.INTAKE_ROTATOR_UP_I, Constants.IntakeConstants.INTAKE_ROTATOR_UP_D);
        intakeRotatorMotorFeedforward = new ArmFeedforward(Constants.IntakeConstants.INTAKE_ROTATOR_MOTOR_S, Constants.IntakeConstants.INTAKE_ROTATOR_MOTOR_G, Constants.IntakeConstants.INTAKE_WHEELS_MOTOR_V);
    }

    public void startIntakeMotor() {
        intakeWheelsMotorBBController.setSetpoint(Constants.IntakeConstants.INTAKE_WHEELS_MOTOR_RPM);
    }

    public void reverseIntakeMotor() {
        intakeWheelsMotorBBController.setSetpoint(Constants.IntakeConstants.INTAKE_WHEELS_MOTOR_RPM * -1);
    }

    public void stopIntakeMotor() {
        intakeWheelsMotorBBController.setSetpoint(0.0);
    }

    public Command startIntakeMotorCommand() {
        return runOnce(() -> startIntakeMotor());
    }

    public Command reverseIntakeMotorCommand() {
        return runOnce(() -> reverseIntakeMotor());
    }

    public Command stopIntakeMotorCommand() {
        return runOnce(() -> stopIntakeMotor());
    }

    public void deployIntake() {
        intakeRotatorMotorPIDController.setSetpoint(Constants.IntakeConstants.INTAKE_COLLECT_POSITION_RADS);
    }

    public Command deployintakeCommand() {
        return runOnce(() -> deployIntake());
    }

    public void retractIntake() {
        intakeRotatorMotorPIDController.setSetpoint(Constants.IntakeConstants.INTAKE_RETRACT_POSITION_RADS);
    }

    public Command retractIntakeCommand() {
        return runOnce(() -> retractIntake());
    }

    public void assistFuelIntake() {
        intakeRotatorMotorPIDController.setSetpoint(Constants.IntakeConstants.INTAKE_MIDDLE_POSITION_RADS);
    }

    public Command assistFuelIntakeCommand() {
        return runOnce(() -> assistFuelIntake()).andThen(new WaitCommand(2)).andThen(deployintakeCommand())
                .andThen(new WaitCommand(2));
    }

    @Override
    public void periodic() {
        intakeWheelsMotor.setVoltage(
            12.0 * intakeWheelsMotorBBController.calculate(intakeWheelsMotor.getEncoder().getVelocity()) 
            + 0.9 * intakeWheelsMotorFeedforward.calculate(intakeWheelsMotorBBController.getSetpoint()));
        intakeRotatorMotor.setVoltage(
            12.0 * intakeRotatorMotorPIDController.calculate(intakeRotatorMotor.getEncoder().getPosition()) 
            + intakeRotatorMotorFeedforward.calculate(intakeRotatorMotorPIDController.getSetpoint(), 0.0));

        SmartDashboard.putNumber("Intake Rotator Motor PID", intakeRotatorMotor.getEncoder().getPosition());
    }
}
