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

    //create motors
    private static SparkFlex intakeWheelsMotor = new SparkFlex(Constants.IntakeConstants.INTAKE_WHEELS_MOTOR_ID,
            MotorType.kBrushless);

    private static SparkFlex intakeRotatorMotor = new SparkFlex(Constants.IntakeConstants.INTAKE_ROTATOR_MOTOR_ID,
            MotorType.kBrushless);

    //create constraints for trapezoidal profile; make sure units of the maxVelocity and maxAcceleration are in encoder ticks or whatever conversion factor used
    private final TrapezoidProfile.Constraints m_Constraints = new TrapezoidProfile.Constraints(9, 6);
    private final TrapezoidProfile.Constraints m_AssistConstraints = new TrapezoidProfile.Constraints(10, 6);

    //create PIDcontroller
    private final ProfiledPIDController intakeRotatorProfiledPIDController;
    private final ProfiledPIDController assistShooterProfiledPIDController;
    //we can call .setConstraints on one of these inside of the method instead of creating two separate ProfiledPIDController objects

    //create state/setpoint
    private static TrapezoidProfile.State goalState = new TrapezoidProfile.State(
            Constants.IntakeConstants.INTAKE_RETRACT_ENCODER_VALUE, 0);

    //feedforward experimentation we never actually used
    private static ArmFeedforward intakeRotationFeedfoward = new ArmFeedforward(.75418, 1.1238, .023506);

    /*SysIdRoutine routine = new SysIdRoutine(new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(intakeRotatorMotor::setVoltage,
                    log -> log.motor("arm").voltage(Volts.of(intakeRotatorMotor.getAppliedOutput() * 12))
                            .angularPosition(Radians.of(intakeRotatorMotor.getEncoder().getPosition() * 2 * Math.PI))
                            .angularVelocity(RadiansPerSecond
                                    .of(intakeRotatorMotor.getEncoder().getVelocity() * 2 * Math.PI / 60)),
                    this, "armSysId"));
*/
    //private static SparkClosedLoopController intakeRotatorPIDController;

    //config object to set current limits
    public static SparkFlexConfig intakeRotatorConfig = new SparkFlexConfig();

    private static SparkClosedLoopController intakeWheelsMotorPIDController;
    public static SparkFlexConfig intakeWheelsMotorConfig = new SparkFlexConfig();

    private static double encoderValue = 0;

    public static boolean useArmRotationAutomaticStatus = true;

    public static boolean useAssist = false;

    public IntakeSubsystem() {

        //set PID on trapezoidal profile

        intakeRotatorProfiledPIDController = new ProfiledPIDController(
                3,
                Constants.IntakeConstants.IntakeRotatorPID.INTAKE_ROTATOR_I,
                0,
                m_Constraints,
                0.02);
        intakeRotatorProfiledPIDController.setTolerance(0.1);
        intakeRotatorProfiledPIDController.setGoal(goalState);

        assistShooterProfiledPIDController = new ProfiledPIDController(
            1.5, 0.008, 0.05, m_AssistConstraints, .02);

        assistShooterProfiledPIDController.setGoal(goalState);



            /* intakeRotatorConfig.closedLoop
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
                .d(0.8, ClosedLoopSlot.kSlot2); */

        intakeRotatorConfig.smartCurrentLimit(60);
        intakeRotatorMotor.configure(intakeRotatorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters,
                com.revrobotics.PersistMode.kNoPersistParameters);

        //intakeRotatorPIDController = intakeRotatorMotor.getClosedLoopController();
        
        //PID for wheels
        intakeWheelsMotorConfig.closedLoop.pid(Constants.IntakeConstants.INTAKE_WHEELS_P,
                Constants.IntakeConstants.INTAKE_WHEELS_I,
                Constants.IntakeConstants.INTAKE_WHEELS_D);
        intakeWheelsMotorConfig.smartCurrentLimit(55);
        intakeWheelsMotor.configure(intakeWheelsMotorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters,
                com.revrobotics.PersistMode.kNoPersistParameters);

        intakeRotatorMotor.getEncoder().setPosition(Constants.IntakeConstants.INTAKE_RETRACT_ENCODER_VALUE);
        intakeWheelsMotorPIDController = intakeWheelsMotor.getClosedLoopController();
    }


    //intake wheels
   

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

    //intake arm
    public void goToPosition(double goalPosition) {
        useAssist = false;
        intakeRotatorProfiledPIDController.reset(new TrapezoidProfile.State(encoderValue, 0));
        goalState = new TrapezoidProfile.State(goalPosition, 0);        
        assistShooterProfiledPIDController.reset(goalState);

        //again, we can just use .setContraints() on one pidcontroller instead of having two--a lot simpler than what we did here
        //call .reset(pull current encoder value) so that if the arm never makes it to goal position encoder, 
        //it won't jerk down or up to that goal before transitioning to next goal
    }

    public Command goToPositionCommand(double goalPosition) {
        return runOnce(() -> goToPosition(goalPosition));
    }

    
    public Command rotateIntakeManualCommand(double speed) {
        return runOnce(() -> rotateIntakeManual(speed));
    }

    public void rotateIntakeManual(double speed) {
        useArmRotationAutomaticStatus = false;
        intakeRotatorMotor.set(speed);
    }

    public void assistShooter(double encoderValue)
    {
        useAssist = true;
        goalState = new TrapezoidProfile.State(encoderValue, 0);
        intakeRotatorProfiledPIDController.reset(goalState);
        //intakeRotatorMotor.setVoltage(assistShooterProfiledPIDController.calculate(encoderValue, goalState));
    }

    public Command assistShooterSlowLiftCommand()
    {
      return runOnce(() -> assistShooter(Constants.IntakeConstants.INTAKE_RETRACT_ENCODER_VALUE));
                //.until(() -> intakeRotatorMotor.getEncoder().getPosition() >= (Constants.IntakeConstants.INTAKE_ASSIST_ENCODER_VALUE - 0.5))
                //.andThen(goToPositionCommand(Constants.IntakeConstants.INTAKE_DEPLOY_ENCODER_VALUE))
                //.andThen(new WaitCommand(1.5));                  
                }

    public Command assistShooterShakeCommand()
    {
        return runOnce(()->assistShooter(Constants.IntakeConstants.INTAKE_ASSIST_ENCODER_VALUE))
        .andThen(new WaitCommand(2.5))
        .andThen(goToPositionCommand(Constants.IntakeConstants.INTAKE_DEPLOY_ENCODER_VALUE))
        .andThen(new WaitCommand(.5));
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

    public void resetTrapezoidalProfile()
    {
        intakeRotatorProfiledPIDController.reset(new TrapezoidProfile.State(encoderValue,0));
        assistShooterProfiledPIDController.reset(new TrapezoidProfile.State(encoderValue,0));

    }

    public Command resetTrapezoidalProfileCommand(){
        return runOnce(()-> resetTrapezoidalProfile());
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
        
        if(useArmRotationAutomaticStatus == true)
        {
            if (useAssist == false) {
            intakeRotatorMotor.setVoltage(intakeRotatorProfiledPIDController.calculate(encoderValue, goalState));

            }
            else {
            intakeRotatorMotor.setVoltage(assistShooterProfiledPIDController.calculate(encoderValue, goalState));

            }
        }
        
               //+ intakeRotationFeedfoward.calculate(intakeRotatorProfiledPIDController.getSetpoint().position * 2 * Math.PI/12,
                       // intakeRotatorProfiledPIDController.getSetpoint().velocity));
        SmartDashboard.putNumber("Intake Rotator Motor Encoder Position", intakeRotatorMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Intake Wheels Motor Encoder RPM", intakeWheelsMotor.getEncoder().getVelocity());

        SmartDashboard.putNumber("Intake Rotation Voltage", intakeRotatorProfiledPIDController.calculate(encoderValue, goalState));
               // + intakeRotationFeedfoward.calculate(intakeRotatorProfiledPIDController.getSetpoint().position * 2 * Math.PI/12,
                       // intakeRotatorProfiledPIDController.getSetpoint().velocity));
        SmartDashboard.putNumber("Intake Rotation Goal Position: ", intakeRotatorProfiledPIDController.getGoal().position);
        SmartDashboard.putNumber("Intake Rotation Arm Velocity", intakeRotatorMotor.getEncoder().getVelocity());
    }
}
