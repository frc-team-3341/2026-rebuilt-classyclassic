package frc.robot.subsystems.intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  // Lower Intake motor
  private SparkFlex intakeBall;
  // Agitates hopper
  private SparkFlex hopperMovement;
  // Raises & Lowers intake
  private SparkMax turnIntake;
  private SparkLimitSwitch fwdLimitSwitch;
  private SparkLimitSwitch revLimitSwitch;

  private SparkClosedLoopController turnIntakeCLC;
  private AbsoluteEncoder turnIntakeAbsoluteEncoder;

  private ShuffleboardTab intakeTab;
  private GenericEntry fwdLimitSwitchEntry;
  private GenericEntry revLimitSwitchEntry;
  private GenericEntry turnEncoderPositionEntry;

  public Intake() {
    intakeBall = new SparkFlex(IntakeConstants.CANIDINTAKEROLLERS, MotorType.kBrushless);
    hopperMovement = new SparkFlex(IntakeConstants.CANIDHOPPER, MotorType.kBrushless);
    turnIntake = new SparkMax(IntakeConstants.CANIDINTAKEARM, MotorType.kBrushless);

    // SoftLimitConfig softLimitConfig = new SoftLimitConfig();

    // softLimitConfig.forwardSoftLimitEnabled(true); //enables the forward soft limit
    // softLimitConfig.reverseSoftLimitEnabled(true); //enables the reverse soft limit
    // softLimitConfig.forwardSoftLimit(0);
    // softLimitConfig.reverseSoftLimit(topSoftLimit);

    revLimitSwitch = turnIntake.getReverseLimitSwitch();
    fwdLimitSwitch = turnIntake.getForwardLimitSwitch();

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
    turnIntakeAbsoluteEncoder = turnIntake.getAbsoluteEncoder();

    fwdLimitSwitchEntry = intakeTab.add("Foward Limit Switch", false).getEntry();
    revLimitSwitchEntry = intakeTab.add("Reverse Limit Switch", false).getEntry();
    turnEncoderPositionEntry =
        intakeTab.add("Turn Encoder Position", turnIntakeAbsoluteEncoder.getPosition()).getEntry();
  }

  // Turn Intake Functions
  public void setTurnIntake(double position) {
    turnIntakeCLC.setSetpoint(position, ControlType.kPosition);
  }

  public void stopTurnIntake() {
    turnIntakeCLC.setSetpoint(0.0, ControlType.kVoltage);
  }

  public boolean canTurnIntake() {
    return false;
  }

  public Command startTurnIntake() {
    return runOnce(
        () -> {
          setTurnIntake(Math.PI / 2);
        });
  }

  public Command reverseTurnIntake() {
    return runOnce(
        () -> {
          setTurnIntake(0);
        });
  }

  // Hopper Functions

  public void setHopper(double power) {
    hopperMovement.set(power);
  }

  public Command startHopper() {
    return runOnce(
        () -> {
          setHopper(IntakeConstants.hopperPower);
        });
  }

  public Command stopHopper() {
    return runOnce(
        () -> {
          setHopper(0);
        });
  }

  // Lower Ball Intake Functions
  public void setIntake(double power) {
    intakeBall.set(power);
  }

  public Command startIntake() {
    return runOnce(
        () -> {
          startHopper();
          setIntake(IntakeConstants.intakeBallPower);
        });
  }

  public Command stopIntake() {
    return runOnce(
        () -> {
          stopHopper();
          setIntake(0);
        });
  }

  public void periodic() {
    fwdLimitSwitchEntry.setBoolean(fwdLimitSwitch.isPressed());
    revLimitSwitchEntry.setBoolean(revLimitSwitch.isPressed());
    turnEncoderPositionEntry.setDouble(turnIntakeAbsoluteEncoder.getPosition());
  }
}
