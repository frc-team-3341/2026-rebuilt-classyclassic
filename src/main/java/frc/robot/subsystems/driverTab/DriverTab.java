package frc.robot.subsystems.driverTab;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class DriverTab extends SubsystemBase {
  // External subsystems
  private Shooter shooter;
  private Intake intake;
  private Drive drive;

  // Shuffleboard Initialization
  private ShuffleboardTab driverTab = Shuffleboard.getTab("Driver Tab");
  // Shooter
  private GenericEntry flywheelSpinning = driverTab.add("Flywheel On?", false).getEntry();
  private GenericEntry canShoot = driverTab.add("Flywheel Within Target?", false).getEntry();
  public GenericEntry flywheelSetpoint = driverTab.add("Flywheel Setpoint RPM", 3000).getEntry();
  // Intake
  private GenericEntry hopperOn = driverTab.add("Hopper On?", false).getEntry();
  private GenericEntry armDeployed = driverTab.add("Arm Deployed?", false).getEntry();
  // Drive
  private Field2d m_field = new Field2d();
  private ComplexWidget robotPose = driverTab.add("Robot Position", m_field);

  public DriverTab(Shooter shooter, Intake intake, Drive drive) {
    this.shooter = shooter;
    this.intake = intake;
    this.drive = drive;

    m_field.setRobotPose(drive.getPose());
  }

  @Override
  public void periodic() {
    flywheelSpinning.setBoolean(shooter.getTargetRPM() > 0);
    canShoot.setBoolean(shooter.canShoot());
    flywheelSetpoint.setDouble(shooter.getTargetRPM());
    hopperOn.setBoolean(intake.getHopperOn());
    armDeployed.setBoolean(intake.getArmDeployed());

    m_field.setRobotPose(drive.getPose());
  }
}
