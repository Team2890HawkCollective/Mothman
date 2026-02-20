package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    private static SparkFlex intakeMotor = new SparkFlex(Constants.IntakeConstants.INTAKE_WHEELS_MOTOR_ID,
            MotorType.kBrushless);

    private static SparkFlex intakeRotatorMotor = new SparkFlex(Constants.IntakeConstants.INTAKE_ROTATOR_MOTOR_ID,
            MotorType.kBrushless);
    private static SparkClosedLoopController intakeRotatorPIDController;
    public static SparkFlexConfig intakeRotatorConfig = new SparkFlexConfig();

    public IntakeSubsystem() {
        intakeRotatorConfig.closedLoop.pid(Constants.IntakeConstants.IntakeRotatorPID.INTAKE_ROTATOR_P,
                Constants.IntakeConstants.IntakeRotatorPID.INTAKE_ROTATOR_I,
                Constants.IntakeConstants.IntakeRotatorPID.INTAKE_ROTATOR_D);
        intakeRotatorMotor.configure(intakeRotatorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kNoPersistParameters);
        intakeRotatorPIDController = intakeRotatorMotor.getClosedLoopController();
    }

    public void startIntakeMotor() {
        intakeMotor.set(Constants.IntakeConstants.INTAKE_WHEELS_MOTOR_SPEED);
    }
    
    public void reverseIntakeMotor() {
        intakeMotor.set(Constants.IntakeConstants.INTAKE_WHEELS_MOTOR_SPEED*-1);
    }

    public void stopIntakeMotor() {
        intakeMotor.set(0);
    }

    public Command startIntakeMotorCommand() {
        return runOnce(() -> startIntakeMotor());
    }

    public Command reverseIntakeMotorCommand () {
        return runOnce(() -> reverseIntakeMotor());
    }

    public Command stopIntakeMotorCommand() {
        return runOnce(() -> stopIntakeMotor());
    }

    public void deployIntake() {
        intakeRotatorPIDController.setSetpoint(Constants.IntakeConstants.INTAKE_COLLECT_ENCODER_VALUE, ControlType.kPosition);
    }

    public Command deployintakeCommand() {
        return runOnce(() -> deployIntake());
    }

    public void retractIntake() {
        intakeRotatorPIDController.setSetpoint(0,ControlType.kPosition);
    }

    public Command retractIntakeCommand() {
        return runOnce(() -> retractIntake());
    }

    @Override
    public void periodic() {
       // Shuffleboard.getTab("Intake Rotator Motor").add("Intake Rotator Motor PID", intakeRotatorMotor.getEncoder().getPosition());
    }
}
