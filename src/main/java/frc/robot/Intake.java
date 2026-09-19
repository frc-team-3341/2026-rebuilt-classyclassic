package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private SparkFlex intakeBall;
  private SparkMax turnIntake;

  private SparkFlexConfig intakeBallConfig;
  private SparkMaxConfig turnIntakeConfig;

  public Intake() {
    intakeBall = new SparkFlex(1, MotorType.kBrushless);
    turnIntake = new SparkMax(2, MotorType.kBrushless);

    intakeBallConfig = new SparkFlexConfig();
    turnIntakeConfig = new SparkMaxConfig();

    intakeBallConfig.closedLoop.pid(0.01, 0, 0);
    turnIntakeConfig.closedLoop.pid(0.01, 0, 0);

    intakeBall.configure(
        intakeBallConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turnIntake.configure(
        turnIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void periodic() {}

  public Command runIntakeBall() {
    return runOnce(
        () -> {
          intakeBall.set(0.25);
        });
  }

  public Command runTurnIntake() {
    return runOnce(
        () -> {
          turnIntake.set(0.25);
        });
  }
}
