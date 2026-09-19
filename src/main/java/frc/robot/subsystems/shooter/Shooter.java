package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Shooter Constants
import frc.robot.Constants.ShooterConstants;;

public class Shooter extends SubsystemBase {
  private SparkFlex flywheelMotor1;
  private SparkFlex flywheelMotor2;
  private SparkFlex topFeeder;
  // CLC = closed loop controller
  private SparkClosedLoopController flyMotor1CLC;
  private SparkClosedLoopController flyMotor2CLC;
  private SparkClosedLoopController topFeederCLC;
  // Encoders
  private RelativeEncoder flyMotor1Encoder;
  private RelativeEncoder flyMotor2Encoder;
  private RelativeEncoder topFeederEncoder;

  private double targetRPM;

  public Shooter() {
    //Flywheel Motor 1/2 and topFeeder setup, configuration using Constants File, and declaration of ClosedLoopControllers and Encoders for use in class functions below
    flywheelMotor1 = new SparkFlex(ShooterConstants.CANIDCONSTANTFLYWHEEL1, MotorType.kBrushless);
    flywheelMotor2 = new SparkFlex(ShooterConstants.CANIDCONSTANTFLYWHEEL2, MotorType.kBrushless);

    topFeeder = new SparkFlex(ShooterConstants.CANIDCONSTANTTOPFEEDER, MotorType.kBrushless);

    flyMotor1CLC = flywheelMotor1.getClosedLoopController();
    flyMotor1Encoder = flywheelMotor1.getEncoder();

    flyMotor2CLC = flywheelMotor2.getClosedLoopController();
    flyMotor2Encoder = flywheelMotor2.getEncoder();

    topFeederCLC = topFeeder.getClosedLoopController();
    topFeederEncoder = topFeeder.getEncoder();

    flywheelMotor1.configure(ShooterConstants.FLYWHEELMOTORCONFIG, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    flywheelMotor2.configure(ShooterConstants.FLYWHEELMOTORCONFIG, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    topFeeder.configure(ShooterConstants.FEEDERMOTORCONFIG, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void resetEncoders() {
    flyMotor1Encoder.setPosition(0);
    flyMotor2Encoder.setPosition(0);
    topFeederEncoder.setPosition(0);
  }

  public void setFlywheelRPM(double rpm) {
    targetRPM = rpm;
    flyMotor1CLC.setSetpoint(targetRPM, ControlType.kVelocity);
    flyMotor2CLC.setSetpoint(targetRPM * -1, ControlType.kVelocity);
  }

  public void stopFlywheel(double rpm) {
    targetRPM = 0;
    flyMotor1CLC.setSetpoint(targetRPM, ControlType.kVelocity);
    flyMotor2CLC.setSetpoint(targetRPM * -1, ControlType.kVelocity);
  }

  public void setTopFeed(double rpm) {
    topFeederCLC.setSetpoint(rpm, ControlType.kVelocity);
  }

  public void startFeed() {
    setTopFeed(ShooterConstants.FEEDERRPM);
  }
  
  public void stopFeed() {
    setTopFeed(0);
  }

  @Override
  public void periodic() {
    // Logging
  }

  public void simulationPeriodic() {}
}
