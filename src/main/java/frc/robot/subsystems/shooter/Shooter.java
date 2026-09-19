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
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private SparkMax motor;
  private SparkMaxConfig motorConfig;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder encoder;

  public Shooter() {
    motor = new SparkMax(1, MotorType.kBrushless);
    closedLoopController = motor.getClosedLoopController();
    encoder = motor.getEncoder();

    motorConfig = new SparkMaxConfig();

    motorConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);

    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.1)
        .i(0)
        .d(0)
        .outputRange(-1, 1)
        // Set PID values for velocity control in slot 1
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
        .feedForward
        // kV is now in Volts, so we multiply by the nominal voltage (12V)
        .kV(12.0 / 5767, ClosedLoopSlot.kSlot1);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  // new Velocity based RPM Change
  public void newVSetpoint(double RPM) {
    closedLoopController.setSetpoint(RPM, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
  }
  // new Position based PID Change
  public void newPSetpoint(double position) {
    closedLoopController.setSetpoint(position, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
  }

  public void changePosRelative(double change) {
    encoder.setPosition(0);
    newPSetpoint(change);
  }

  @Override
  public void periodic() {}

  public void simulationPeriodic() {}
}
