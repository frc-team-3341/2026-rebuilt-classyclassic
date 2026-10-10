package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

public class GyroIOSim implements GyroIO {
  private boolean connected = true;
  private Rotation2d yawPosition = Rotation2d.kZero;
  private double yawVelocityRadPerSec = 0.0;

  public void updateSim(double angularVelocityRadPerSec, double dt) {
    yawVelocityRadPerSec = angularVelocityRadPerSec;
    yawPosition = yawPosition.plus(new Rotation2d(angularVelocityRadPerSec * dt));
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = connected;
    inputs.yawPosition = yawPosition;
    inputs.yawVelocityRadPerSec = yawVelocityRadPerSec;
    inputs.odometryYawTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryYawPositions = new Rotation2d[] {yawPosition};
  }
}
