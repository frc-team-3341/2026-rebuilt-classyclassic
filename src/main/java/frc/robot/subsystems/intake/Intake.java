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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  // Lower Intake motor
  private SparkFlex intakeRollers;
  // Agitates hopper
  private SparkFlex hopperMovement;
  // Raises & Lowers intake
  private SparkMax intakeArm;
  private SparkLimitSwitch fwdLimitSwitch;
  private SparkLimitSwitch revLimitSwitch;
  private boolean intakeArmState = false; //false for retracted true for deployed

  private SparkClosedLoopController intakeArmCLC;
  private AbsoluteEncoder intakeArmAbsoluteEncoder;

  private ShuffleboardTab intakeTab;
  private GenericEntry fwdLimitSwitchEntry;
  private GenericEntry revLimitSwitchEntry;
  private GenericEntry turnEncoderPositionEntry;

  public Intake() {
    intakeRollers = new SparkFlex(IntakeConstants.CANIDINTAKEROLLERS, MotorType.kBrushless);
    hopperMovement = new SparkFlex(IntakeConstants.CANIDHOPPER, MotorType.kBrushless);
    intakeArm = new SparkMax(IntakeConstants.CANIDINTAKEARM, MotorType.kBrushless);

    // SoftLimitConfig softLimitConfig = new SoftLimitConfig();
    // softLimitConfig.forwardSoftLimitEnabled(true); //enables the forward soft limit
    // softLimitConfig.reverseSoftLimitEnabled(true); //enables the reverse soft limit
    // softLimitConfig.forwardSoftLimit(0);
    // softLimitConfig.reverseSoftLimit(topSoftLimit);

    revLimitSwitch = intakeArm.getReverseLimitSwitch(); // arm up pos
    fwdLimitSwitch = intakeArm.getForwardLimitSwitch(); // arm down pos

    intakeRollers.configure(
        IntakeConstants.intakeRollersConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    hopperMovement.configure(
        IntakeConstants.intakeRollersConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    intakeArm.configure(
        IntakeConstants.intakeArmConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    intakeArmCLC = intakeArm.getClosedLoopController();
    intakeArmAbsoluteEncoder = intakeArm.getAbsoluteEncoder();

    intakeTab = Shuffleboard.getTab("intake");
    fwdLimitSwitchEntry = intakeTab.add("Foward Limit Switch", false).getEntry();
    revLimitSwitchEntry = intakeTab.add("Reverse Limit Switch", false).getEntry();
    turnEncoderPositionEntry =
        intakeTab.add("Turn Encoder Position", intakeArmAbsoluteEncoder.getPosition()).getEntry();
  }

  // Turn Intake Functions
  // might need to pivot to using velocity setpoints for now
  public void setIntakeArm(double velocity) {
    // retract velocity -600
    // deploy velocity 200
    intakeArmCLC.setSetpoint(velocity, ControlType.kVelocity);
  }

  public void stopintakeArm() {
    intakeArmCLC.setSetpoint(0.0, ControlType.kVoltage);
  }

  public boolean canIntakeArm() {
    return false;
  }

  public Command deployIntakeArm() {
    return runOnce(
        () -> {
          setIntakeArm(200);
        });
  }

  public Command retractIntakeArm() {
    return runOnce(
        () -> {
          setIntakeArm(-600);
        });
  }
  
  public void toggleIntakeArm() {
    if (intakeArmState) {
      deployIntakeArm();
    } else {
      retractIntakeArm();
    }
    intakeArmState = !intakeArmState;
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
    intakeRollers.set(power);
  }

  public Command startIntake() {
    return runOnce(
        () -> {
          // startHopper();
          setIntake(0.75);
        });
  }

  public Command stopIntake() {
    return runOnce(
        () -> {
          // stopHopper();
          setIntake(0);
        });
  }

  public void periodic() {
    fwdLimitSwitchEntry.setBoolean(fwdLimitSwitch.isPressed());
    revLimitSwitchEntry.setBoolean(revLimitSwitch.isPressed());
    turnEncoderPositionEntry.setDouble(intakeArmAbsoluteEncoder.getPosition());
  }
}
