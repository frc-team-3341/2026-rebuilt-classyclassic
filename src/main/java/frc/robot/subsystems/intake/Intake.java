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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class Intake extends SubsystemBase {
  // Lower Intake motor
  private SparkFlex intakeRollers;
  // Agitates hopper
  private SparkFlex hopperMovement;
  // Raises & Lowers intake
  private SparkMax intakeArm;
  private SparkLimitSwitch fwdLimitSwitch;
  private SparkLimitSwitch revLimitSwitch;

  private SparkClosedLoopController intakeArmCLC;
  private AbsoluteEncoder intakeArmAbsoluteEncoder;

  private SparkClosedLoopController intakeRollersCLC;
  private SparkClosedLoopController hopperRollersCLC;

  private ShuffleboardTab intakeTab;
  private GenericEntry fwdLimitSwitchEntry;
  private GenericEntry revLimitSwitchEntry;
  private GenericEntry turnEncoderPositionEntry;
  private GenericEntry intakeRollersRPMEntry;
  private GenericEntry hopperRollersRPMEntry;
  private GenericEntry armDeployedEntry;

  private boolean hopperToggle;
  private boolean armDeployed;

  public Intake() {
    intakeRollers = new SparkFlex(IntakeConstants.CANIDINTAKEROLLERS, MotorType.kBrushless);
    hopperMovement = new SparkFlex(IntakeConstants.CANIDHOPPER, MotorType.kBrushless);
    intakeArm = new SparkMax(IntakeConstants.CANIDINTAKEARM, MotorType.kBrushless);

    revLimitSwitch = intakeArm.getReverseLimitSwitch(); // arm up pos
    fwdLimitSwitch = intakeArm.getForwardLimitSwitch(); // arm down pos

    intakeRollers.configure(
        IntakeConstants.intakeRollersConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    hopperMovement.configure(
        IntakeConstants.hopperMovementConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    intakeArm.configure(
        IntakeConstants.intakeArmConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    intakeArmCLC = intakeArm.getClosedLoopController();
    intakeArmAbsoluteEncoder = intakeArm.getAbsoluteEncoder();

    intakeRollersCLC = intakeRollers.getClosedLoopController();
    hopperRollersCLC = hopperMovement.getClosedLoopController();
    // Arbitrary please change
    if (intakeArmAbsoluteEncoder.getPosition() > 0.4) {
      armDeployed = false;
    } else {
      armDeployed = true;
    }

    intakeTab = Shuffleboard.getTab("intake");
    armDeployedEntry = intakeTab.add("Arm Deployed", false).getEntry();
    fwdLimitSwitchEntry = intakeTab.add("Foward Limit Switch", false).getEntry();
    revLimitSwitchEntry = intakeTab.add("Reverse Limit Switch", false).getEntry();
    turnEncoderPositionEntry = intakeTab.add("Turn Encoder Position", intakeArmAbsoluteEncoder.getPosition()).getEntry();
    intakeRollersRPMEntry = intakeTab.add("intakeRollersRPM", 1500).getEntry();
    hopperRollersRPMEntry = intakeTab.add("hopperRollersRPM", 1500).getEntry();
    armDeployedEntry = intakeTab.add("Arm Deployed", false).getEntry();

    hopperToggle = false;
    if (intakeArmAbsoluteEncoder.getPosition() >= 0.4) {
        armDeployed = false;
    } else {
      armDeployed = true;
    }
  }

  // Turn Intake Functions
  // might need to pivot to using velocity setpoints for now
  public void setIntakeArm(double velocity) {
    // retract velocity -600
    // deploy velocity 200
    intakeArmCLC.setSetpoint(velocity, ControlType.kVelocity);
  }

  public void stopintakeArm() {
    intakeArmCLC.setSetpoint(0.0, ControlType.kVelocity);
  }

  public boolean canIntakeArm() {
    Boolean armMotorSpinning = intakeArmAbsoluteEncoder.getVelocity() > 1;
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

  public Command toggleIntakeArm() {
    return Commands.runOnce(() -> {
      if (armDeployed) {
        deployIntakeArm();
      } else {
        retractIntakeArm();
      }
    });
  }

  // Hopper Functions

  public void setHopper(double rpm) {
    hopperRollersCLC.setSetpoint(rpm, ControlType.kVelocity);
  }

  public Command toggleHopper() {
    if (hopperToggle) {
      hopperToggle = false;
      return runOnce(
          () -> {
            stopHopper();
          });
    } else {
      hopperToggle = true;
      return runOnce(
          () -> {
            startHopper();
          });
    }
  }

  public Command startHopper() {
    return runOnce(
        () -> {
          setHopper(hopperRollersRPMEntry.getDouble(1500));
        });
  }

  public Command stopHopper() {
    return runOnce(
        () -> {
          setHopper(0);
        });
  }

  public Command reverseHopper() {
    return runOnce(
        () -> {
          double test = intakeRollersRPMEntry.getDouble(1500);
          setHopper(-test);
        });
  }

  // Lower Ball Intake Functions
  public void setIntake(double rpm) {
    intakeRollersCLC.setSetpoint(rpm, ControlType.kVelocity);
  }

  public Command startIntake() {
    return runOnce(
        () -> {
          // startHopper();
          setIntake(intakeRollersRPMEntry.getDouble(1500));
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
    if (intakeArmAbsoluteEncoder.getPosition() > 0.4) {
      armDeployed = false;
    } else {
      armDeployed = true;
    }

    fwdLimitSwitchEntry.setBoolean(fwdLimitSwitch.isPressed());
    revLimitSwitchEntry.setBoolean(revLimitSwitch.isPressed());
    turnEncoderPositionEntry.setDouble(intakeArmAbsoluteEncoder.getPosition());
    armDeployedEntry.setBoolean(armDeployed);
  }
}
