package frc.robot.subsystems.intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  // Intake motor
  private SparkFlex intakeBall;
  // Agitates hopper
  private SparkFlex hopperMovement;
  // Raises & Lowers intake
  private SparkMax turnIntake;

  private boolean motorIsOn;

  public Intake() {
    intakeBall = new SparkFlex(IntakeConstants.CANIDINTAKEBALL, MotorType.kBrushless);
    hopperMovement = new SparkFlex(IntakeConstants.CANIDHOPPER, MotorType.kBrushless);
    turnIntake = new SparkMax(IntakeConstants.CANIDINTAKETURN, MotorType.kBrushless);

    motorIsOn = false;

    intakeBall.configure(
        IntakeConstants.intakeBallConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turnIntake.configure(
        IntakeConstants.turnIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void periodic() {}

  public Command runIntakeBall() {
    return runOnce(
        () -> {
          intakeBall.set(0.25);
        });
  }

  public Command reverseIntakeBall() {
    return runOnce(
        () -> {
          intakeBall.set(-0.25);
        });
  }

  public Command runTurnIntake() {
    return runOnce(
        () -> {
          turnIntake.set(0.25);
        });
  }

  public Command stopTurnIntake() {
    return runOnce(
        () -> {
          turnIntake.set(0);
        });
  }

  public Command stopIntakeBall() {
    return runOnce(
        () -> {
          intakeBall.set(0);
        });
  }

  public void setMotorOn(boolean val) {
    motorIsOn = val;
  }

  public boolean getMotorOn() {
    return motorIsOn;
  }

  public Command setMotorCommandOn(boolean val) {
    return runOnce(
        () -> {
          motorIsOn = val;
        });
  }

  public Command keepOn() {
    return runOnce(
        () -> {
          if (getMotorOn()) {
            setMotorOn(false);
          } else {
            setMotorOn(true);
          }
        });
  }
}
