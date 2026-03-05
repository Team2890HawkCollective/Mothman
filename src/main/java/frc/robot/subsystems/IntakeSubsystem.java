package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
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

    private final TrapezoidProfile.Constraints m_Constraints = new TrapezoidProfile.Constraints(5, 10);
    private final ProfiledPIDController intakeRotatorProfiledPIDController;

    private static SparkClosedLoopController intakeRotatorPIDController;
    public static SparkFlexConfig intakeRotatorConfig = new SparkFlexConfig();

    private static SparkClosedLoopController intakeWheelsMotorPIDController;
    public static SparkFlexConfig intakeWheelsMotorConfig = new SparkFlexConfig();

    public static DutyCycleEncoder intakeRotatorEncoder = new DutyCycleEncoder(1);

    private static double encoderValue = intakeRotatorEncoder.get();

    public IntakeSubsystem() {
       intakeRotatorProfiledPIDController = new ProfiledPIDController(
            Constants.IntakeConstants.IntakeRotatorPID.INTAKE_ROTATOR_P,
            Constants.IntakeConstants.IntakeRotatorPID.INTAKE_ROTATOR_I,
            Constants.IntakeConstants.IntakeRotatorPID.INTAKE_ROTATOR_D, m_Constraints);
        intakeRotatorProfiledPIDController.setTolerance(0.05);
        //intakeRotatorProfiledPIDController.setGoal(Constants.IntakeConstants.INTAKE_THROUGHBORE_ENCODER_RETRACT);

        intakeRotatorConfig.closedLoop
                //Slot 0
                .p(Constants.IntakeConstants.IntakeRotatorPID.INTAKE_ROTATOR_P)
                .i(Constants.IntakeConstants.IntakeRotatorPID.INTAKE_ROTATOR_I)
                .d(Constants.IntakeConstants.IntakeRotatorPID.INTAKE_ROTATOR_D)

                //Slot 1
                .p(.05, ClosedLoopSlot.kSlot1)
                .i(0, ClosedLoopSlot.kSlot1)
                .d(.05, ClosedLoopSlot.kSlot1)

                //Slot 2
                .p(1,ClosedLoopSlot.kSlot2)
                .i(.001, ClosedLoopSlot.kSlot2)
                .d(0.1, ClosedLoopSlot.kSlot2);

        intakeRotatorMotor.configure(intakeRotatorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters,
                com.revrobotics.PersistMode.kNoPersistParameters);
        intakeRotatorConfig.smartCurrentLimit(40);
        intakeRotatorPIDController = intakeRotatorMotor.getClosedLoopController();
        

        intakeWheelsMotorConfig.closedLoop.pid(Constants.IntakeConstants.INTAKE_MOTOR_P,
                Constants.IntakeConstants.INTAKE_MOTOR_I,
                Constants.IntakeConstants.INTAKE_MOTOR_D);
        intakeWheelsMotor.configure(intakeWheelsMotorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters,
                com.revrobotics.PersistMode.kNoPersistParameters);
        intakeRotatorMotor.getEncoder().setPosition(-5);
        intakeWheelsMotorPIDController = intakeWheelsMotor.getClosedLoopController();
    }

    /* public void goToPosition(double goalPosition) {
        intakeRotatorProfiledPIDController.setGoal(goalPosition);
        //double pidVal = intakeRotatorProfiledPIDController.calculate(encoderValue, goalPosition);  
    } 

    public Command goToPositionCommand(double goalPosition) {
        return runOnce(() -> goToPosition(goalPosition));
    } 
        */

    public void rotateIntake(double speed)
    {
        intakeRotatorMotor.set(speed);
    }

    public Command rotateIntakeCommand(double speed)
    {
        return runOnce(()->intakeRotatorMotor.set(speed));
    }

    public void startIntakeMotor() {
        intakeWheelsMotorPIDController.setSetpoint(Constants.IntakeConstants.INTAKE_WHEELS_MOTOR_RPM,
                ControlType.kVelocity);
    }

    public void reverseIntakeMotor() {
        intakeWheelsMotorPIDController.setSetpoint(Constants.IntakeConstants.INTAKE_WHEELS_MOTOR_RPM * -1,
                ControlType.kVelocity);
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

    /* public Command assistFuelIntakeCommand(double deployedPosition, double assistPosition) {
        return runOnce(() -> goToPositionCommand(assistPosition).andThen(new WaitCommand(1.5))
                .andThen(goToPositionCommand(deployedPosition)).andThen(new WaitCommand(1.5)));
    } */

    public static void resetIntakeRotationEncoder() {
        intakeRotatorMotor.getEncoder().setPosition(-5);
    }
    
     public void deployIntake() {
      intakeRotatorPIDController.setSetpoint(Constants.IntakeConstants.
      INTAKE_COLLECT_ENCODER_VALUE, ControlType.kPosition, ClosedLoopSlot.kSlot0);
      }
      
      public Command deployIntakeCommand() {
      return runOnce(() -> deployIntake());
      }
      
     public void retractIntake() {
      intakeRotatorPIDController.setSetpoint(Constants.IntakeConstants.
      INTAKE_RETRACT_ENCODER_VALUE, ControlType.kPosition, ClosedLoopSlot.kSlot1);
      }
     
      public Command retractIntakeCommand() {
      return runOnce(() -> retractIntake());
      }
      
      public void assistFuelIntake() {
     intakeRotatorPIDController.setSetpoint(Constants.IntakeConstants.
      INTAKE_MIDDLE_ENCODER_VALUE,
      ControlType.kPosition, ClosedLoopSlot.kSlot2);
      }
      
      public Command assistFuelIntakeCommand() {
      return runOnce(() -> assistFuelIntake()).andThen(new
      WaitCommand(1)).andThen(deployIntakeCommand())
      .andThen(new WaitCommand(1));
      }
    

    @Override
    public void periodic() {
        //encoderValue = intakeRotatorEncoder.get();
        //intakeRotatorMotor.setVoltage(intakeRotatorProfiledPIDController.calculate(encoderValue) * -12);
        SmartDashboard.putNumber("Intake Rotator Motor PID", intakeRotatorMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Intake Rotator Encoder Value", intakeRotatorEncoder.get());
        SmartDashboard.putNumber("Voltage Deploy", intakeRotatorProfiledPIDController.calculate(encoderValue, Constants.IntakeConstants.INTAKE_THROUGHBORE_ENCODER_DEPLOY));
        SmartDashboard.putNumber("Voltage Retract", intakeRotatorProfiledPIDController.calculate(encoderValue, Constants.IntakeConstants.INTAKE_THROUGHBORE_ENCODER_RETRACT));

    }
}
