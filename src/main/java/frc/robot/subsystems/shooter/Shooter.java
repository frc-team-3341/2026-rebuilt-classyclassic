package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private SparkFlex flywheelMotor1;
  private SparkFlex flywheelMotor2;
  private SparkFlex topFeeder;
  private SparkClosedLoopController flyMotor1CLC;
  private SparkClosedLoopController flyMotor2CLC;
  private SparkClosedLoopController topFeederCLC;
  private RelativeEncoder flyMotor1Encoder;
  private RelativeEncoder flyMotor2Encoder;
  private RelativeEncoder topFeederEncoder;

  public Shooter() {
    //Flywheel Motor 1/2 and topFeeder setup, configuration using Constants File, and declaration of ClosedLoopControllers and Encoders for use in class functions below
    flywheelMotor1 = new SparkFlex(CANIDCONSTANTFLYWHEEL1, MotorType.kBrushless);
    flywheelMotor2 = new SparkFlex(CANIDCONSTANTFLYWHEEL2, MotorType.kBrushless);
    topFeeder = new SparkFlex(CANIDCONSTANTTOPFEEDER, MotorType.kBrushless);
    flyMotor1CLC = flywheelMotor1.getClosedLoopController();
    flyMotor1Encoder = flywheelMotor1.getEncoder();
    flyMotor2CLC = flywheelMotor2.getClosedLoopController();
    flyMotor2Encoder = flywheelMotor2.getEncoder();
    topFeederCLC = topFeeder.getClosedLoopController();
    topFeederEncoder = topFeeder.getEncoder();
    flywheelMotor1.configure(FLYWHEELMOTORCONFIGCONSTANT, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    flywheelMotor2.configure(FLYWHEELMOTORCONFIGCONSTANT, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    topFeeder.configure(FEEDERMOTORCONFIGCONSTANT, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  // new Velocity based RPM Change
  public void newVSetpoint(SparkClosedLoopController motorCLC, double RPM) {
    motorCLC.setSetpoint(RPM, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
  }
  // new Position based PID Change
  public void newPSetpoint(SparkClosedLoopController motorCLC, double position) {
    motorCLC.setSetpoint(position, ControlType.kPosition, ClosedLoopSlot.kSlot1);
  }

  public void changePosRelative(SparkClosedLoopController motorCLC, RelativeEncoder motorEncoder, double change) {
    motorEncoder.setPosition(0);
    newPSetpoint(motorCLC, change);
  }

  @Override
  public void periodic() {}

  public void simulationPeriodic() {}
}
