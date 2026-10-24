package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  // Other Subsystems

  private SparkFlex flywheelMotor1;
  private SparkFlex flywheelMotor2;
  private SparkFlex topFeeder;
  private SparkFlex bottomFeeder;
  // CLC = closed loop controller
  private SparkClosedLoopController flyMotor1CLC;
  private SparkClosedLoopController topFeederCLC;
  private SparkClosedLoopController bottomFeederCLC;
  // Encoders
  private RelativeEncoder flyMotor1Encoder;
  private RelativeEncoder flyMotor2Encoder;
  private RelativeEncoder topFeederEncoder;
  private RelativeEncoder bottomFeederEncoder;

  private double targetRPM;

  private boolean flywheelToggle = false;

  private int flywheelSetpointRPM;
  private int feedSetpointRPM;
  private int feedBSetpointRPM;
  private int feedReverseSetpointRPM;
  private int feedBReverseSetpointRPM;

  // Shuffleboard
  ShuffleboardTab shooterTab;
  GenericEntry targetSendableRPM;
  GenericEntry encoderSendableRPM;
  GenericEntry canShootSendable;
  GenericEntry flywheelSendableRPM;
  GenericEntry feedSendableRPM;
  GenericEntry feedBSendableRPM;
  GenericEntry feedSendableRPMReverse;
  GenericEntry feedBSendableRPMReverse;

  public Shooter() {

    // Import external subsystems

    // Flywheel Motor #1 is the main motor for the flywheel.
    flywheelMotor1 = new SparkFlex(ShooterConstants.CANIDFLYWHEEL1, MotorType.kBrushless);
    // Flywheel Motor #2 is the follower motor for the flywheel.
    flywheelMotor2 = new SparkFlex(ShooterConstants.CANIDFLYWHEEL2, MotorType.kBrushless);

    topFeeder = new SparkFlex(ShooterConstants.CANIDTOPFEEDER, MotorType.kBrushless);

    bottomFeeder = new SparkFlex(ShooterConstants.CANIDBOTTOMFEEDER, MotorType.kBrushless);

    flyMotor1CLC = flywheelMotor1.getClosedLoopController();
    flyMotor1Encoder = flywheelMotor1.getEncoder();

    topFeederCLC = topFeeder.getClosedLoopController();
    topFeederEncoder = topFeeder.getEncoder();
    bottomFeederEncoder = bottomFeeder.getEncoder();
    bottomFeederCLC = bottomFeeder.getClosedLoopController();

    flywheelMotor1.configure(
        ShooterConstants.FLYWHEELMOTORCONFIG,
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

    // Use follower config for secondary flywheel.
    flywheelMotor2.configure(
        ShooterConstants.FOLLOWERMOTORCONFIG,
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

    topFeeder.configure(
        ShooterConstants.FEEDERMOTORCONFIG.inverted(ShooterConstants.REVERSETOPFEEDER),
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

    bottomFeeder.configure(
        ShooterConstants.FEEDERMOTORCONFIG.inverted(ShooterConstants.REVERSEBOTTOMFEEDER),
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

    // Shuffleboard configs
    shooterTab = Shuffleboard.getTab("Shooter");
    targetSendableRPM = shooterTab.add("Flywheel Target RPM", 0).getEntry();
    encoderSendableRPM =
        shooterTab.add("Flywheel Encoder RPM", flyMotor1Encoder.getVelocity()).getEntry();

    flywheelSendableRPM = shooterTab.add("Flywheel Setpoint RPM", 3000).getEntry();
    feedSendableRPM = shooterTab.add("Feed Top Setpoint RPM", 4000).getEntry();
    feedBSendableRPM = shooterTab.add("Feed Bottom Setpoint RPM", 4000).getEntry();
    canShootSendable = shooterTab.add("Shooter CanShoot?", false).getEntry();
    feedSendableRPMReverse = shooterTab.add("Feed Top Reverse Setpoint RPM", -1000).getEntry();
    feedBSendableRPMReverse = shooterTab.add("Feed Bottom Reverse Setpoint RPM", -1000).getEntry();
  }

  public void resetEncoders() {
    flyMotor1Encoder.setPosition(0);
    flyMotor2Encoder.setPosition(0);
    topFeederEncoder.setPosition(0);
    bottomFeederEncoder.setPosition(0);
  }

  // Shoot Functions

  public void setFlywheelRPM(double rpm) {
    targetRPM = rpm;
    flyMotor1CLC.setSetpoint(rpm, ControlType.kVelocity);
  }

  public Command stopFlywheel() {
    return runOnce(
        () -> {
          targetRPM = 0;
          setFlywheelRPM(targetRPM);
        });
  }

  public Command startFlywheel() {
    return runOnce(
        () -> {
          setFlywheelRPM(flywheelSetpointRPM);
        });
  }

  public void backupShoot() {
    // Placeholder speed
    setFlywheelRPM(ShooterConstants.BACKUPSHOOTERRPM);
  }

  public Command toggleFlywheel() {
    if (flywheelToggle) {
      flywheelToggle = !flywheelToggle;
      return startFlywheel();
    } else {
      flywheelToggle = !flywheelToggle;
      return stopFlywheel();
    }
  }

  // Feed Functions

  public void setTopFeed(double rpm) {
    topFeederCLC.setSetpoint(rpm, ControlType.kVelocity);
  }

  public void setBottomFeed(double rpm) {
    bottomFeederCLC.setSetpoint(rpm, ControlType.kVelocity);
  }

  public void startFeed() {
    if (canShoot() || true) { // fix me
      setTopFeed(feedSetpointRPM);
      setBottomFeed(feedBSetpointRPM); // verify this again had to unreverse bottome feeder
    }
  }

  public void reverseFeed() {
    setTopFeed(feedReverseSetpointRPM);
    setBottomFeed(feedBReverseSetpointRPM);
  }

  public void stopFeed() {
    setTopFeed(0);
    setBottomFeed(0);
  }

  // Utility Functions

  public double getShootRPM(double distance) {
    // Returns RPM from distance to target in meters
    return ShooterConstants.shooterMap.get(distance);
  }

  // if this isnt working then it might be better to not use it for now
  public boolean canShoot() {
    // Checks if flywheel speed is within acceptable range of target speed (200 rpm)
    Boolean flywheelReady = Math.abs(flyMotor1Encoder.getVelocity() - targetRPM) <= 200;
    // Checks if flywheel is spinning
    Boolean flywheelSpinning = targetRPM > 10;

    // Check if all conditions are true
    if (flywheelReady && flywheelSpinning) {
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    // Logging
    targetSendableRPM.setDouble(targetRPM);
    encoderSendableRPM.setDouble(flyMotor1Encoder.getVelocity());
    flywheelSetpointRPM = Math.toIntExact(flywheelSendableRPM.getInteger(3000));
    feedSetpointRPM = Math.toIntExact(feedSendableRPM.getInteger(4000));
    feedBSetpointRPM = Math.toIntExact(feedBSendableRPM.getInteger(4000));
    canShootSendable.setBoolean(canShoot());
    feedReverseSetpointRPM = Math.toIntExact(feedSendableRPMReverse.getInteger(-1000));
    feedBReverseSetpointRPM = Math.toIntExact(feedBSendableRPMReverse.getInteger(-1000));
  }

  public void simulationPeriodic() {}
}
