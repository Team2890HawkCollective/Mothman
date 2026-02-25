package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class ShooterSubsystem extends SubsystemBase {
    private static SparkFlex centerShooterMotor = new SparkFlex(Constants.ShooterConstants.CENTER_SHOOTER_MOTOR_ID,
            MotorType.kBrushless);

    private static SparkFlex leftShooterMotor = new SparkFlex(Constants.ShooterConstants.LEFT_SHOOTER_MOTOR_ID,
            MotorType.kBrushless);

    private static SparkFlex rightShooterMotor = new SparkFlex(Constants.ShooterConstants.RIGHT_SHOOTER_MOTOR_ID,
            MotorType.kBrushless);

    private static SparkFlex indexerAndRampMotor = new SparkFlex(Constants.ShooterConstants.INDEXER_MOTOR_ID,
            MotorType.kBrushless);

    public static SparkFlexConfig centerShooterMotorConfig = new SparkFlexConfig();
    private static BangBangController centerShooterMotorBBController = new BangBangController();
    private static SimpleMotorFeedforward centerShooterMotorFeedforward;

    public static SparkFlexConfig leftShooterMotorConfig = new SparkFlexConfig();
    private static BangBangController leftShooterMotorBBController = new BangBangController();
    private static SimpleMotorFeedforward leftShooterMotorFeedforward;

    public static SparkFlexConfig rightShooterMotorConfig = new SparkFlexConfig();
    private static BangBangController rightShooterMotorBBController = new BangBangController();
    private static SimpleMotorFeedforward rightShooterMotorFeedforward;

    public static SparkFlexConfig indexerAndRampMotorConfig = new SparkFlexConfig();
    private static PIDController indexerAndRampMotorPIDController;

    public ShooterSubsystem() {
        centerShooterMotorConfig.smartCurrentLimit(60);
        centerShooterMotor.configure(centerShooterMotorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters,
            com.revrobotics.PersistMode.kNoPersistParameters);
        centerShooterMotorFeedforward = new SimpleMotorFeedforward(Constants.ShooterConstants.CENTER_MOTOR_S, Constants.ShooterConstants.CENTER_MOTOR_V);
    
        leftShooterMotorConfig.smartCurrentLimit(60);
        leftShooterMotor.configure(leftShooterMotorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters,
            com.revrobotics.PersistMode.kNoPersistParameters);
        leftShooterMotorFeedforward = new SimpleMotorFeedforward(Constants.ShooterConstants.LEFT_MOTOR_S, Constants.ShooterConstants.LEFT_MOTOR_V);
    
        rightShooterMotorConfig.smartCurrentLimit(60);
        rightShooterMotor.configure(rightShooterMotorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters,
            com.revrobotics.PersistMode.kNoPersistParameters);
        rightShooterMotorFeedforward = new SimpleMotorFeedforward(Constants.ShooterConstants.RIGHT_MOTOR_S, Constants.ShooterConstants.RIGHT_MOTOR_V);

        indexerAndRampMotorConfig.smartCurrentLimit(60);
        indexerAndRampMotor.configure(indexerAndRampMotorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters,
            com.revrobotics.PersistMode.kNoPersistParameters);
        indexerAndRampMotorPIDController = new PIDController(Constants.ShooterConstants.INDEXER_MOTOR_P, Constants.ShooterConstants.INDEXER_MOTOR_I, Constants.ShooterConstants.INDEXER_MOTOR_D);
    }


    //private static SparkMax leftActuatorMotor = new SparkMax(Constants.ShooterConstants.LEFT_ACTUATOR_PWM_PORT,
      //      MotorType.kBrushless);

    //private static SparkMax rightActuatorMotor = new SparkMax(Constants.ShooterConstants.RIGHT_ACTUATOR_PWM_PORT,
            //MotorType.kBrushless);

    //private static AnalogPotentiometer leftPotentiometer = new AnalogPotentiometer(0, 1, 0);
    //private static AnalogPotentiometer rightPotentiometer = new AnalogPotentiometer(0, 1, 0);

    private static double currentPotentiometerPosition; // might need second value for the right potentiometer

    public void setShooterMotorsRPM() {
        centerShooterMotorBBController.setSetpoint(Constants.ShooterConstants.SHOOTER_RPM);
        leftShooterMotorBBController.setSetpoint(Constants.ShooterConstants.SHOOTER_RPM);
        rightShooterMotorBBController.setSetpoint(Constants.ShooterConstants.SHOOTER_RPM);
    }

    //test individual motor code
    public void setLeftShooterMotorRPM() {
        leftShooterMotorBBController.setSetpoint(Constants.ShooterConstants.SHOOTER_RPM);
    }
    public Command testLeftShooterCommand() {
        return runOnce(() -> setLeftShooterMotorRPM());
    }

    public void setRightShooterMotorRPM() {
        rightShooterMotorBBController.setSetpoint(Constants.ShooterConstants.SHOOTER_RPM);
    }
    public Command testRightShooterCommand() {
        return runOnce(() -> setRightShooterMotorRPM());
    }

    public void setCenterShooterMotorRPM() {
        centerShooterMotorBBController.setSetpoint(Constants.ShooterConstants.SHOOTER_RPM);
    }
    public Command testCenterShooterCommand() {
        return runOnce(() -> setCenterShooterMotorRPM());
    }

    public void stopLeftShooterMotorRPM() {
        leftShooterMotorBBController.setSetpoint(0.0);
    }
    public Command stopLeftShooterCommand() {
        return runOnce(() -> stopLeftShooterMotorRPM());
    }

    public void stopCenterShooterMotorRPM() {
        centerShooterMotorBBController.setSetpoint(0.0);
    }
    public Command stopCenterShooterCommand() {
        return runOnce(() -> stopCenterShooterMotorRPM());
    }

    public void stopRightShooterMotorRPM() {
        rightShooterMotorBBController.setSetpoint(0.0);
    }
    public Command stopRightShooterCommand() {
        return runOnce(() -> stopRightShooterMotorRPM());
    }
    

    public double getShooterMotorRPM() {
        return rightShooterMotor.getEncoder().getVelocity();
    }

    public void setIndexerAndRampMotorRPM() {
        indexerAndRampMotorPIDController.setSetpoint(Constants.ShooterConstants.INDEXER_AND_RAMP_MOTOR_RPM);
    }

    public void reverseIndexerAndRampMotorRPM() {
        indexerAndRampMotorPIDController.setSetpoint(Constants.ShooterConstants.INDEXER_AND_RAMP_MOTOR_RPM * -1);
    }

    public Command reverseIndexerAndRampMotorRPMCommand() {
        return runOnce(() -> reverseIndexerAndRampMotorRPM());
    }

    public Command setIndexerAndRampMotorRPMCommand() {
        return runOnce(() -> setIndexerAndRampMotorRPM());
    }

    public Command stopIndexerAndRampMotorCommand() {
        return runOnce(()-> indexerAndRampMotorPIDController.setSetpoint(0.0));
    }

    /*public Command shootFuelCommand() {
        return runOnce(() -> setShooterMotorsRPM())
                .until(() -> {return (getShooterMotorRPM() <= Constants.ShooterConstants.SHOOTER_RPM * 0.9);})
                .andThen(() -> setIndexerAndRampMotorRPM()); 
    } */

    
      public Command shootFuelCommand() {
      return runOnce(() -> setShooterMotorsRPM()).andThen(new WaitCommand(2))
      .andThen(() -> setIndexerAndRampMotorRPM());
      }; 
     

    public void stopShooters() {
        centerShooterMotorBBController.setSetpoint(0.0);
        leftShooterMotorBBController.setSetpoint(0.0);
        rightShooterMotorBBController.setSetpoint(0.0);
        indexerAndRampMotorPIDController.setSetpoint(0.0);
    }

    public Command stopShooterCommand() {
        return runOnce(() -> stopShooters());
    }

    public void moveActuator(double desiredPotentiometerPosition) {
        if (desiredPotentiometerPosition > currentPotentiometerPosition) {
            //TODO: Test for positive or negative power
            //leftActuatorMotor.set(0.1);
            //rightActuatorMotor.set(0.1);
        } else {
            //leftActuatorMotor.set(-0.1);
            //rightActuatorMotor.set(-0.1);
        }
    }

    public void stopActuator() {
        //leftActuatorMotor.set(0);
        //rightActuatorMotor.set(0);
    }

    public Command moveActuatorCommand(double desiredPotentiometerPosition) {
        return run(() -> moveActuator(desiredPotentiometerPosition))
                .until(() -> currentPotentiometerPosition == currentPotentiometerPosition)
                .andThen(() -> stopActuator());
    }

    @Override
    public void periodic() {
        centerShooterMotor.setVoltage(
            12.0 * centerShooterMotorBBController.calculate(centerShooterMotor.getEncoder().getVelocity())
            + 0.9 * centerShooterMotorFeedforward.calculate(centerShooterMotorBBController.getSetpoint()));
        leftShooterMotor.setVoltage(
            12.0 * leftShooterMotorBBController.calculate(leftShooterMotor.getEncoder().getVelocity())
            + 0.9 * leftShooterMotorFeedforward.calculate(leftShooterMotorBBController.getSetpoint()));
        rightShooterMotor.setVoltage(
            12.0 * rightShooterMotorBBController.calculate(rightShooterMotor.getEncoder().getVelocity())
            + 0.9 * rightShooterMotorFeedforward.calculate(rightShooterMotorBBController.getSetpoint()));
        indexerAndRampMotor.setVoltage(
            12.0 * indexerAndRampMotorPIDController.calculate(indexerAndRampMotor.getEncoder().getVelocity()));

        SmartDashboard.putNumber("Shooter Velocity", leftShooterMotor.getEncoder().getVelocity());
        /* SmartDashboard.putNumber("Left Potentiometer Distance", leftPotentiometer.get());
        SmartDashboard.putNumber("Right Potentiometer Distance", rightPotentiometer.get());
        currentPotentiometerPosition = leftPotentiometer.get(); */
    }
}
