package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkClosedLoopController;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


public class Intake extends SubsystemBase {
  // Lower Intake motor
  private SparkFlex intakeBall;
  // Agitates hopper
  private SparkFlex hopperMovement;
  // Raises & Lowers intake
  private SparkMax turnIntake;
  private SparkClosedLoopController turnIntakeCLC;

  private ShuffleboardTab intakeTab;

  public Intake() {
    intakeBall = new SparkFlex(IntakeConstants.CANIDINTAKEROLLERS, MotorType.kBrushless);
    hopperMovement = new SparkFlex(IntakeConstants.CANIDHOPPER, MotorType.kBrushless);
    turnIntake = new SparkMax(IntakeConstants.CANIDINTAKEARM, MotorType.kBrushless);

    intakeBall.configure(
        IntakeConstants.intakeBallConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    
    hopperMovement.configure(
        IntakeConstants.intakeBallConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    
    turnIntake.configure(
        IntakeConstants.turnIntakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    
    turnIntakeCLC = turnIntake.getClosedLoopController();
    
  }

  public void periodic() {}

  // Turn Intake Functions
  public void setTurnIntake(double rpm) {
    turnIntakeCLC.setSetpoint(rpm, ControlType.kVelocity);
  }

  public void stopTurnIntake() {
    setTurnIntake(0.0);
  }

  public boolean canTurnIntake() {return false;}

  public Command startTurnIntake() {
    return runOnce(() -> {
      setTurnIntake(200);
    });
  }

  public Command reverseTurnIntake() {
    return runOnce(() -> {
      setTurnIntake(-200);
    });
  }

  // Hopper Functions

  public void setHopper(double power) {
    hopperMovement.set(power);
  }

  public Command startHopper() {
    return runOnce(() -> {
      setHopper(IntakeConstants.hopperPower);
    });
  }

  public Command stopHopper() {
    return runOnce(() -> {
      setHopper(0);
    });
  }

  // Lower Ball Intake Functions
  public void setIntake(double power) {
    intakeBall.set(power);
  }

  public Command startIntake() {
    return runOnce(() -> {
      startHopper();
      setIntake(IntakeConstants.intakeBallPower);
    });
  }

  public Command stopIntake() {
    return runOnce(() -> {
      stopHopper();
      setIntake(0);
    });
  }
}