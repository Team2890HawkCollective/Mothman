package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    private static TalonFX climberMotor = new TalonFX(Constants.ClimberConstants.CLIMB_MOTOR_ID);
    private static Servo climberRatchet = new Servo(Constants.ClimberConstants.RATCHET_PWM_PORT);

    public void liftRobot() {
        climberMotor.set(Constants.ClimberConstants.CLIMBER_SPEED_DUTY_CYCLE);
    }

    public void lowerRobot() {
        climberMotor.set(Constants.ClimberConstants.CLIMBER_SPEED_DUTY_CYCLE * -1);
    }

    public void stopClimber() {
        climberMotor.set(0);
    }

    public Command liftRobotCommand() {
        return runOnce(() -> toggleRatchet(true)).andThen(() -> liftRobot());
    }

    public Command lowerRobotCommand() {
        return runOnce(() -> toggleRatchet(false)).andThen(() -> lowerRobot());
    }

    public Command stopClimberCommand() {
        return runOnce(() -> stopClimber());
    }

    public static void toggleRatchet(boolean toggle) {
      if (toggle == true) {
        climberRatchet.setAngle(Constants.ClimberConstants.RATCHET_LOCK_ANGLE_DEGREES);
      } else
        climberRatchet.setAngle(Constants.ClimberConstants.RATCHET_UNLOCK_ANGLE_DEGREES);
    }

  public Command toggleRatchetCommand(boolean toggle) {
    return runOnce(() -> toggleRatchet(toggle));
  }

  @Override
  public void periodic()
  {
    SmartDashboard.putNumber("Ratchet Position" , climberRatchet.getAngle());
  }
}
