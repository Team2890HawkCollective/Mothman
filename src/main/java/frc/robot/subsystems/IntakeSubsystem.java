package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;

public class IntakeSubsystem extends SubsystemBase {

    private static SparkFlex intakeWheelsMotor = new SparkFlex(Constants.IntakeConstants.INTAKE_WHEELS_MOTOR_ID,
            MotorType.kBrushless);

    private static SparkFlex intakeRotatorMotor = new SparkFlex(Constants.IntakeConstants.INTAKE_ROTATOR_MOTOR_ID,
            MotorType.kBrushless);
    private static SparkClosedLoopController intakeRotatorPIDController;
    public static SparkFlexConfig intakeRotatorConfig = new SparkFlexConfig();

    private static SparkClosedLoopController intakeWheelsMotorPIDController;
    public static SparkFlexConfig intakeWheelsMotorConfig = new SparkFlexConfig();

    public IntakeSubsystem() {
        intakeRotatorConfig.closedLoop
        .p(Constants.IntakeConstants.IntakeRotatorPID.INTAKE_ROTATOR_P)
        .i(Constants.IntakeConstants.IntakeRotatorPID.INTAKE_ROTATOR_I)
        .d(Constants.IntakeConstants.IntakeRotatorPID.INTAKE_ROTATOR_D)

        .p(.06, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1);
        intakeRotatorMotor.configure(intakeRotatorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters,
                com.revrobotics.PersistMode.kNoPersistParameters);
        intakeRotatorPIDController = intakeRotatorMotor.getClosedLoopController();

        intakeWheelsMotorConfig.closedLoop.pid(Constants.IntakeConstants.INTAKE_MOTOR_P,
                Constants.IntakeConstants.INTAKE_MOTOR_I,
                Constants.IntakeConstants.INTAKE_MOTOR_D);
        intakeWheelsMotor.configure(intakeWheelsMotorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters,
                com.revrobotics.PersistMode.kNoPersistParameters);
        intakeWheelsMotorPIDController = intakeWheelsMotor.getClosedLoopController();
    }

    public void startIntakeMotor() {
        intakeWheelsMotorPIDController.setSetpoint(Constants.IntakeConstants.INTAKE_WHEELS_MOTOR_RPM, ControlType.kVelocity);
    }

    public void reverseIntakeMotor() {
        intakeWheelsMotorPIDController.setSetpoint(Constants.IntakeConstants.INTAKE_WHEELS_MOTOR_RPM * -1, ControlType.kVelocity);
    }

    public void stopIntakeMotor() {
        intakeWheelsMotor.set(0);
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
        intakeRotatorPIDController.setSetpoint(Constants.IntakeConstants.INTAKE_COLLECT_ENCODER_VALUE, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public Command deployintakeCommand() {
        return runOnce(() -> deployIntake());
    }

    public void retractIntake() {
        intakeRotatorPIDController.setSetpoint(Constants.IntakeConstants.INTAKE_RETRACT_ENCODER_VALUE, ControlType.kPosition, ClosedLoopSlot.kSlot1);
    }

    public Command retractIntakeCommand() {
        return runOnce(() -> retractIntake());
    }

    public void assistFuelIntake() {
        intakeRotatorPIDController.setSetpoint(Constants.IntakeConstants.INTAKE_MIDDLE_ENCODER_VALUE,
                ControlType.kPosition);
    }

    public Command assistFuelIntakeCommand() {
        return runOnce(() -> assistFuelIntake()).andThen(new WaitCommand(2)).andThen(deployintakeCommand())
                .andThen(new WaitCommand(2));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Rotator Motor PID", intakeRotatorMotor.getEncoder().getPosition());
    }
}
