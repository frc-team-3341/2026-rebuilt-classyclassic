package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private SparkFlex flywheelMotor1;
  private SparkFlex flywheelMotor2;
  private SparkFlex topFeeder;
  // CLC = closed loop controller
  private SparkClosedLoopController flyMotor1CLC;
  private SparkClosedLoopController topFeederCLC;
  // Encoders
  private RelativeEncoder flyMotor1Encoder;
  private RelativeEncoder flyMotor2Encoder;
  private RelativeEncoder topFeederEncoder;

  private double targetRPM;
  private double revRPM = 1;

  public Shooter() {
    // Flywheel Motor #1 is the main motor for the flywheel.
    flywheelMotor1 = new SparkFlex(ShooterConstants.CANIDCONSTANTFLYWHEEL1, MotorType.kBrushless);
    // Flywheel Motor #2 is the follower motor for the flywheel.
    flywheelMotor2 = new SparkFlex(ShooterConstants.CANIDCONSTANTFLYWHEEL2, MotorType.kBrushless);

    topFeeder = new SparkFlex(ShooterConstants.CANIDCONSTANTTOPFEEDER, MotorType.kBrushless);

    flyMotor1CLC = flywheelMotor1.getClosedLoopController();
    flyMotor1Encoder = flywheelMotor1.getEncoder();

    topFeederCLC = topFeeder.getClosedLoopController();
    topFeederEncoder = topFeeder.getEncoder();

    flywheelMotor1.configure(
        ShooterConstants.FLYWHEELMOTORCONFIG,
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);
    flywheelMotor2.configure(
        ShooterConstants.FOLLOWERMOTORCONFIG,
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);
    topFeeder.configure(
        ShooterConstants.FEEDERMOTORCONFIG,
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);
    
    if (ShooterConstants.REVERSEFLYWHEEL) {
      revRPM = -1;
    }
  }

  public void resetEncoders() {
    flyMotor1Encoder.setPosition(0);
    flyMotor2Encoder.setPosition(0);
    topFeederEncoder.setPosition(0);
  }

  // Flywheel Functions

  public void setFlywheelRPM(double rpm) {
    targetRPM = rpm * revRPM;
    flyMotor1CLC.setSetpoint(targetRPM, ControlType.kVelocity);
  }

  public void stopFlywheel(double rpm) {
    targetRPM = 0;
    setFlywheelRPM(targetRPM);
  }


  // Feed Functions
  public void setTopFeed(double rpm) {
    topFeederCLC.setSetpoint(rpm, ControlType.kVelocity);
  }

  public void startFeed() {
    if (canShoot()) {
      setTopFeed(ShooterConstants.FEEDERRPM);
    }
  }

  public void stopFeed() {
    setTopFeed(0);
  }

  public boolean canShoot() {
    // Checks if flywheel speed is within acceptable range of target speed (200 rpm)
    Boolean flywheelReady = Math.abs(flyMotor1Encoder.getVelocity() - targetRPM) <= 200;

    // Check if all conditions are true
    if (flywheelReady) {
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    // Logging
  }

  public void simulationPeriodic() {}
}
