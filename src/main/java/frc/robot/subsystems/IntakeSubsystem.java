package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.fasterxml.jackson.databind.RuntimeJsonMappingException;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;

public class IntakeSubsystem extends SubsystemBase {

    private static SparkFlex intakeWheelsMotor = new SparkFlex(Constants.IntakeConstants.INTAKE_WHEELS_MOTOR_ID,
            MotorType.kBrushless);

    private static SparkFlex intakeRotatorMotor = new SparkFlex(Constants.IntakeConstants.INTAKE_ROTATOR_MOTOR_ID,
            MotorType.kBrushless);

    private final TrapezoidProfile.Constraints m_Constraints = new TrapezoidProfile.Constraints(8, 3);
    private final TrapezoidProfile.Constraints m_AssistConstraints = new TrapezoidProfile.Constraints(1, .5);

    private final ProfiledPIDController intakeRotatorProfiledPIDController;
    private final ProfiledPIDController assistShooterProfiledPIDController;

    private static TrapezoidProfile.State goalState = new TrapezoidProfile.State(
            Constants.IntakeConstants.INTAKE_RETRACT_ENCODER_VALUE, 0);

    private static TrapezoidProfile.State assistShooterState = new TrapezoidProfile.State(Constants.IntakeConstants.INTAKE_ASSIST_ENCODER_VALUE, 0);

    private static ArmFeedforward intakeRotationFeedfoward = new ArmFeedforward(.75418, 1.1238, .023506, .043444);

    /*SysIdRoutine routine = new SysIdRoutine(new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(intakeRotatorMotor::setVoltage,
                    log -> log.motor("arm").voltage(Volts.of(intakeRotatorMotor.getAppliedOutput() * 12))
                            .angularPosition(Radians.of(intakeRotatorMotor.getEncoder().getPosition() * 2 * Math.PI))
                            .angularVelocity(RadiansPerSecond
                                    .of(intakeRotatorMotor.getEncoder().getVelocity() * 2 * Math.PI / 60)),
                    this, "armSysId"));
*/
    //private static SparkClosedLoopController intakeRotatorPIDController;
    public static SparkFlexConfig intakeRotatorConfig = new SparkFlexConfig();

    private static SparkClosedLoopController intakeWheelsMotorPIDController;
    public static SparkFlexConfig intakeWheelsMotorConfig = new SparkFlexConfig();

    public static DutyCycleEncoder intakeRotatorEncoder = new DutyCycleEncoder(1);

    private static double encoderValue = intakeRotatorEncoder.get();

    public static boolean useArmRotationAutomaticStatus = true;

    public IntakeSubsystem() {
        intakeRotatorProfiledPIDController = new ProfiledPIDController(
                3,
                Constants.IntakeConstants.IntakeRotatorPID.INTAKE_ROTATOR_I,
                .1,
                m_Constraints,
                0.02);
        intakeRotatorProfiledPIDController.setTolerance(0.15);
        intakeRotatorProfiledPIDController.setGoal(goalState);

        assistShooterProfiledPIDController = new ProfiledPIDController(
            3, 0, .1, m_AssistConstraints, .02);

        assistShooterProfiledPIDController.setTolerance(.15);
                assistShooterProfiledPIDController.setGoal(assistShooterState);



        intakeRotatorConfig.closedLoop
                // Slot 0
                .p(Constants.IntakeConstants.IntakeRotatorPID.INTAKE_ROTATOR_P)
                .i(Constants.IntakeConstants.IntakeRotatorPID.INTAKE_ROTATOR_I)
                .d(Constants.IntakeConstants.IntakeRotatorPID.INTAKE_ROTATOR_D)

                // Slot 1
                .p(.05, ClosedLoopSlot.kSlot1)
                .i(0, ClosedLoopSlot.kSlot1)
                .d(.05, ClosedLoopSlot.kSlot1)

                // Slot 2
                .p(.8, ClosedLoopSlot.kSlot2)
                .i(.0, ClosedLoopSlot.kSlot2)
                .d(0.8, ClosedLoopSlot.kSlot2);

        intakeRotatorConfig.smartCurrentLimit(70);
        intakeRotatorMotor.configure(intakeRotatorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters,
                com.revrobotics.PersistMode.kNoPersistParameters);

        //intakeRotatorPIDController = intakeRotatorMotor.getClosedLoopController();

        intakeWheelsMotorConfig.closedLoop.pid(Constants.IntakeConstants.INTAKE_MOTOR_P,
                Constants.IntakeConstants.INTAKE_MOTOR_I,
                Constants.IntakeConstants.INTAKE_MOTOR_D);
        intakeWheelsMotorConfig.smartCurrentLimit(70);
        intakeWheelsMotor.configure(intakeWheelsMotorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters,
                com.revrobotics.PersistMode.kNoPersistParameters);

        intakeRotatorMotor.getEncoder().setPosition(Constants.IntakeConstants.INTAKE_RETRACT_ENCODER_VALUE);
        intakeWheelsMotorPIDController = intakeWheelsMotor.getClosedLoopController();
    }

    public void goToPosition(double goalPosition) {
        goalState = new TrapezoidProfile.State(goalPosition, 0);
        assistShooterState = new TrapezoidProfile.State(goalPosition, 0);
        assistShooterProfiledPIDController.reset(assistShooterState);
        intakeRotatorMotor.setVoltage(intakeRotatorProfiledPIDController.calculate(encoderValue, goalState));

    }

    public Command goToPositionCommand(double goalPosition) {
        return run(() -> goToPosition(goalPosition));
    }

    public void rotateIntakeManual(double speed) {
        useArmRotationAutomaticStatus = false;
        intakeRotatorMotor.set(speed);
    }

    public Command rotateIntakeManualCommand(double speed) {
        return runOnce(() -> rotateIntakeManual(speed));
    }

    public void startIntakeMotorWheels(double speed) {
        //intakeWheelsMotor.set(speed);
        intakeWheelsMotorPIDController.setSetpoint(speed,
               ControlType.kVelocity);
    }

    public void reverseIntakeWheels() {
        intakeWheelsMotorPIDController.setSetpoint(Constants.IntakeConstants.INTAKE_WHEELS_MOTOR_RPM_FAST * -1,
                ControlType.kVelocity);
    }

    public void stopIntakeWheels() {
        intakeWheelsMotor.set(0);
    }

    public Command startIntakeWheelsCommand(double speed) {
        return runOnce(() -> startIntakeMotorWheels(speed));
    }

    public Command reverseIntakeWheelsCommand() {
        return runOnce(() -> reverseIntakeWheels());
    }

    public Command stopIntakeWheelsCommand() {
        return runOnce(() -> stopIntakeWheels());
    }

    public void assistShooter()
    {
        goalState = new TrapezoidProfile.State(Constants.IntakeConstants.INTAKE_ASSIST_ENCODER_VALUE, 0);

        assistShooterState = new TrapezoidProfile.State(Constants.IntakeConstants.INTAKE_ASSIST_ENCODER_VALUE, 0);

        intakeRotatorMotor.setVoltage(assistShooterProfiledPIDController.calculate(encoderValue, assistShooterState));
    }

    public Command assistShooterCommand()
    {
      return run(() -> assistShooter());
      }
     

    public static void resetIntakeRotationEncoder() {
        intakeRotatorMotor.getEncoder().setPosition(Constants.IntakeConstants.INTAKE_RETRACT_ENCODER_VALUE);
    }

    public static void setUseArmRotationAutomaticStatus(boolean status)
    {
        useArmRotationAutomaticStatus = status;
    }

    public Command setUseArmRotationAutomaticStatusCommand(boolean status)
    {
        return runOnce(()->setUseArmRotationAutomaticStatus(status));
    }

    /*public void deployIntake() {
        intakeRotatorPIDController.setSetpoint(Constants.IntakeConstants.INTAKE_COLLECT_ENCODER_VALUE,
                ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void deployIntakeAssist() {
        intakeRotatorPIDController.setSetpoint(Constants.IntakeConstants.INTAKE_COLLECT_ENCODER_VALUE,
                ControlType.kPosition, ClosedLoopSlot.kSlot2);
    }

    public Command deployIntakeCommand() {
        return runOnce(() -> deployIntake());
    }

    public void retractIntake() {
        intakeRotatorPIDController.setSetpoint(Constants.IntakeConstants.INTAKE_RETRACT_ENCODER_VALUE,
                ControlType.kPosition, ClosedLoopSlot.kSlot1);
    }

    public Command retractIntakeCommand() {
        return runOnce(() -> retractIntake());
    }

    public void assistFuelIntake() {
        intakeRotatorPIDController.setSetpoint(Constants.IntakeConstants.INTAKE_MIDDLE_ENCODER_VALUE,
                ControlType.kPosition, ClosedLoopSlot.kSlot2);
    }

    public Command assistFuelIntakeCommand() {
        return runOnce(() -> assistFuelIntake()).andThen(new WaitCommand(1.5)).andThen(deployIntakeCommand())
                .andThen(new WaitCommand(1.5));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }*/

    @Override
    public void periodic() {
        encoderValue = intakeRotatorMotor.getEncoder().getPosition();
        
        /*if(useArmRotationAutomaticStatus == true)
        {
            intakeRotatorMotor.setVoltage(intakeRotatorProfiledPIDController.calculate(encoderValue, goalState));
        }*/
        
               //+ intakeRotationFeedfoward.calculate(intakeRotatorProfiledPIDController.getSetpoint().position * 2 * Math.PI/12,
                       // intakeRotatorProfiledPIDController.getSetpoint().velocity));
        SmartDashboard.putNumber("Intake Rotator Motor Encoder Position", intakeRotatorMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Intake Wheels Motor Encoder RPM", intakeRotatorMotor.getEncoder().getVelocity());

        SmartDashboard.putNumber("Intake Rotator Throughbore Encoder Value", intakeRotatorEncoder.get());
        SmartDashboard.putNumber("Intake Rotation Voltage", intakeRotatorProfiledPIDController.calculate(encoderValue, goalState));
               // + intakeRotationFeedfoward.calculate(intakeRotatorProfiledPIDController.getSetpoint().position * 2 * Math.PI/12,
                       // intakeRotatorProfiledPIDController.getSetpoint().velocity));
        SmartDashboard.putNumber("Intake Rotation Goal Position: ", intakeRotatorProfiledPIDController.getGoal().position);
    }
}
