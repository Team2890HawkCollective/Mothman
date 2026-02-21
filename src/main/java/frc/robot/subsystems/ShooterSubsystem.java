package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.controls.Follower;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

public class ShooterSubsystem extends SubsystemBase {
    private static SparkFlex centerShooterMotor = new SparkFlex(Constants.ShooterConstants.CENTER_SHOOTER_MOTOR_ID,
            MotorType.kBrushless);

    //private static SparkFlex leftShooterMotor = new SparkFlex(Constants.ShooterConstants.LEFT_SHOOTER_MOTOR_ID,
      //      MotorType.kBrushless);

    private static SparkFlex rightShooterMotor = new SparkFlex(Constants.ShooterConstants.RIGHT_SHOOTER_MOTOR_ID,
            MotorType.kBrushless);

    private static SparkFlex indexerAndRampMotor = new SparkFlex(Constants.ShooterConstants.INDEXER_MOTOR_ID,
            MotorType.kBrushless);

    private static SparkClosedLoopController centerShooterMotorPIDController;
    public static SparkFlexConfig centerShooterMotorConfig = new SparkFlexConfig();

    //private static SparkClosedLoopController leftShooterMotorPIDController;
    //public static SparkFlexConfig leftShooterMotorConfig = new SparkFlexConfig();

    private static SparkClosedLoopController rightShooterMotorPIDController;
    public static SparkFlexConfig rightShooterMotorConfig = new SparkFlexConfig();

    private static SparkClosedLoopController indexerAndRampMotorPIDController;
    public static SparkFlexConfig indexerAndRampMotorConfig = new SparkFlexConfig();

    public ShooterSubsystem() {
        centerShooterMotorConfig.closedLoop.pid(Constants.ShooterConstants.SHOOTER_MOTOR_P,
                Constants.ShooterConstants.SHOOTER_MOTOR_I,
                Constants.ShooterConstants.SHOOTER_MOTOR_D);
        centerShooterMotor.configure(centerShooterMotorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters,
                com.revrobotics.PersistMode.kNoPersistParameters);
        centerShooterMotorPIDController = centerShooterMotor.getClosedLoopController();
    
    /*leftShooterMotorConfig.closedLoop.pid(Constants.ShooterConstants.SHOOTER_MOTOR_P,
                Constants.ShooterConstants.SHOOTER_MOTOR_I,
                Constants.ShooterConstants.SHOOTER_MOTOR_D);
        leftShooterMotor.configure(leftShooterMotorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters,
                com.revrobotics.PersistMode.kNoPersistParameters);
        leftShooterMotorPIDController = leftShooterMotor.getClosedLoopController();
    */
    rightShooterMotorConfig.closedLoop.pid(Constants.ShooterConstants.SHOOTER_MOTOR_P,
                Constants.ShooterConstants.SHOOTER_MOTOR_I,
                Constants.ShooterConstants.SHOOTER_MOTOR_D);
        rightShooterMotor.configure(rightShooterMotorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters,
                com.revrobotics.PersistMode.kNoPersistParameters);
        rightShooterMotorPIDController = rightShooterMotor.getClosedLoopController();

        indexerAndRampMotorConfig.closedLoop.pid(Constants.ShooterConstants.SHOOTER_MOTOR_P,
                Constants.ShooterConstants.SHOOTER_MOTOR_I,
                Constants.ShooterConstants.SHOOTER_MOTOR_D);
        indexerAndRampMotor.configure(indexerAndRampMotorConfig, com.revrobotics.ResetMode.kNoResetSafeParameters,
                com.revrobotics.PersistMode.kNoPersistParameters);
        indexerAndRampMotorPIDController = indexerAndRampMotor.getClosedLoopController();
    }


    //private static SparkMax leftActuatorMotor = new SparkMax(Constants.ShooterConstants.LEFT_ACTUATOR_PWM_PORT,
           // MotorType.kBrushless);

    //private static SparkMax rightActuatorMotor = new SparkMax(Constants.ShooterConstants.RIGHT_ACTUATOR_PWM_PORT,
            //MotorType.kBrushless);

    //private static AnalogPotentiometer leftPotentiometer = new AnalogPotentiometer(0, 1, 0);
    //private static AnalogPotentiometer rightPotentiometer = new AnalogPotentiometer(0, 1, 0);

    private static double currentPotentiometerPosition; // might need second value for the right potentiometer

    public void setShooterMotorsRPM() {
        centerShooterMotorPIDController.setSetpoint(Constants.ShooterConstants.SHOOTER_RPM,ControlType.kVelocity);
        //leftShooterMotorPIDController.setSetpoint(Constants.ShooterConstants.SHOOTER_RPM,ControlType.kVelocity);
        rightShooterMotorPIDController.setSetpoint(Constants.ShooterConstants.SHOOTER_RPM,ControlType.kVelocity);
    }

    public double getShooterMotorRPM() {
        return rightShooterMotor.getEncoder().getVelocity();
    }

    public void setIndexerAndRampMotorRPM() {
        indexerAndRampMotorPIDController.setSetpoint(Constants.ShooterConstants.INDEXER_AND_RAMP_MOTOR_RPM, ControlType.kVelocity);
    }

    /* public Command shootFuelCommand() {
        return run(() -> setShooterMotorsRPM())
                .until(() -> {
                    return (getShooterMotorRPM() >= Constants.ShooterConstants.SHOOTER_RPM);
                })
                .andThen(() -> setIndexerAndRampMotorRPM()); 
    } */

    
      public Command shootFuelCommand() {
      return runOnce(() -> setShooterMotorsRPM()).andThen(new WaitCommand(2))
      .andThen(() -> setIndexerAndRampMotorRPM());
      }; 
     

    public void stopShooters() {

        centerShooterMotor.set(0);
        //leftShooterMotor.set(0);
        rightShooterMotor.set(0);
        indexerAndRampMotor.set(0);

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
        /* SmartDashboard.putNumber("Left Potentiometer Distance", leftPotentiometer.get());
        SmartDashboard.putNumber("Right Potentiometer Distance", rightPotentiometer.get());
        currentPotentiometerPosition = leftPotentiometer.get(); */
    }
}
