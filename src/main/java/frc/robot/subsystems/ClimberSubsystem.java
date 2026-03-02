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
  private static SparkFlex climberMotor = new SparkFlex(Constants.ClimberConstants.CLIMB_MOTOR_ID,
      MotorType.kBrushless);
  private static SparkClosedLoopController climberMotorPIDController;
  public static SparkFlexConfig climberMotorConfig = new SparkFlexConfig();

  public ClimberSubsystem() {
    climberMotorConfig.closedLoop.pid(Constants.ClimberConstants.CLIMBER_PID_P,
        Constants.ClimberConstants.CLIMBER_PID_I,
        Constants.ClimberConstants.CLIMBER_PID_D);
    climberMotorConfig.smartCurrentLimit(80);
    climberMotorConfig.openLoopRampRate(0);
    climberMotorConfig.closedLoopRampRate(0);
    climberMotor.configure(climberMotorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters,
        com.revrobotics.PersistMode.kNoPersistParameters);
    climberMotorPIDController = climberMotor.getClosedLoopController();
  }

  public void liftRobot() {
    climberMotorPIDController.setSetpoint(Constants.ClimberConstants.CLIMBER_LIFTED_SETPOINT_VALUE, ControlType.kPosition);
  }

  public void lowerRobot() {
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
  }
}
