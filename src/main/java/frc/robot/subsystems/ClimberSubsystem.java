package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  // private static TalonFX climberMotor = new
  // TalonFX(Constants.ClimberConstants.CLIMB_MOTOR_ID);
  private static SparkFlex climberMotor = new SparkFlex(Constants.ClimberConstants.CLIMB_MOTOR_ID, //makes the motor inside of code
      MotorType.kBrushless);

  private static SparkClosedLoopController climberMotorPIDController; //creates a PID controller without initialization

  private static SparkFlexConfig climberMotorConfig = new SparkFlexConfig(); //creates a config object that we will configure inside constructor

  public ClimberSubsystem() {
    climberMotorConfig.closedLoop.pid(Constants.ClimberConstants.CLIMBER_PID_P, //PID is a way on how to tell the robot to move
        Constants.ClimberConstants.CLIMBER_PID_I,
        Constants.ClimberConstants.CLIMBER_PID_D); //creates PID values inside of our config object by calling constants

    climberMotorConfig.smartCurrentLimit(80); //set max voltage limit on the motor
    climberMotorConfig.openLoopRampRate(0); //time it takes to accelerate from 0 speed to desired/max speed
    climberMotorConfig.closedLoopRampRate(0);

    climberMotor.configure(climberMotorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters, //apply the configs we initialized in lines above to the actual motor itself
        com.revrobotics.PersistMode.kNoPersistParameters);

    climberMotorPIDController = climberMotor.getClosedLoopController();
  }

  public void setClimberSpeed(double speed) {
    climberMotor.set(speed);
  }

  public Command setClimberSpeedCommand(double speed) {
    return runOnce(() -> setClimberSpeed(speed));
  }

  public void liftRobot() {
    climberMotorPIDController.setSetpoint(Constants.ClimberConstants.CLIMBER_LIFTED_SETPOINT_VALUE, ControlType.kPosition);
  }

  public static void lowerRobot() {
    climberMotorPIDController.setSetpoint(Constants.ClimberConstants.CLIMBER_LOWERED_SETPOINT_VALUE, ControlType.kPosition);
  }

  public void stopClimber() {
    climberMotor.set(0);
  }

  public Command liftRobotCommand() {
    return runOnce(() -> liftRobot());
  } 

  public Command lowerRobotCommand() {
    return runOnce(() -> lowerRobot());
  }

  public Command stopClimberCommand() {
    return runOnce(() -> stopClimber());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber Motor Encoder", climberMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Climber motor power", climberMotor.get());
  }
}
