// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private Shooter shooter;
  private SwerveDriveSimulation driveSimulation;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Intake
  private Intake robotIntake;

  // Drive Mode driveMode
  private int driveMode;

  @SuppressWarnings("unused")
  private final Vision vision;

  // private final GyroIOSim gyroIO = new GyroIOSim(null);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        createIntake();
        createShooter();
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(camera0Name, robotToCamera0),
                new VisionIOPhotonVision(camera1Name, robotToCamera1));
        // new VisionIOPhotonVisionSim(camera2Name, robotToCamera2, drive::getPose)); // once 3rd
        // camera is added
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        var SwerveDriveConfig =
            DriveTrainSimulationConfig.Default()
                .withRobotMass(Kilograms.of(robotMassKg))
                .withBumperSize(Meters.of(bumperLengthMeters), Meters.of(bumperWidthMeters))
                .withCustomModuleTranslations(moduleTranslations)
                .withGyro(COTS.ofNav2X())
                .withSwerveModule(
                    new SwerveModuleSimulationConfig(
                        driveGearbox,
                        turnGearbox,
                        driveMotorReduction,
                        turnMotorReduction,
                        Volts.of(driveSimKs),
                        Volts.of(turnSimFrictionVolts),
                        Meters.of(wheelRadiusMeters),
                        KilogramSquareMeters.of(turnSimMOI),
                        wheelCOF));

        driveSimulation =
            new SwerveDriveSimulation(SwerveDriveConfig, new Pose2d(2.0, 2.0, Rotation2d.kZero));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSim(driveSimulation.getModules()[0]),
                new ModuleIOSim(driveSimulation.getModules()[1]),
                new ModuleIOSim(driveSimulation.getModules()[2]),
                new ModuleIOSim(driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);

        createShooter();
        createIntake();
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
                // new VisionIOPhotonVisionSim(camera2Name, robotToCamera2, drive::getPose));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        createIntake();
        createShooter();
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();

    // vision = new Vision(drive::addVisionMeasurement);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Reset gyro to 0° when X button is pressed
    controller
        .x()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // controller.x().onTrue(Commands.runOnce(() -> AutoCommands.HubCentric(drive)));
    // below line to be replaced with flywheel spin function later
    // controller.a().onTrue(Commands.runOnce(() -> shooter.newPSetpoint(0.0), shooter));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void simulationPeriodic() {
    if (driveSimulation != null) {
      SimulatedArena.getInstance().simulationPeriodic();
      drive.setSimulationPose(driveSimulation.getSimulatedDriveTrainPose());
    }
  }

  public void createShooter() {
    shooter = new Shooter();
    // Feed Function
    Trigger feedTrigger = controller.rightTrigger();
    feedTrigger.debounce(1);
    feedTrigger.onTrue(Commands.runOnce(() -> shooter.startFeed()));
    feedTrigger.onFalse(Commands.runOnce(() -> shooter.stopFeed()));

    // Start Flywheel
    Trigger flywheelTrigger = controller.rightBumper();
    flywheelTrigger.debounce(1);
    flywheelTrigger.toggleOnTrue(Commands.runOnce(() -> shooter.shoot()));
    flywheelTrigger.toggleOnFalse(Commands.runOnce(() -> shooter.stopFlywheel()));

    // Trigger feedBTrigger = controller.leftTrigger();
    // feedBTrigger.debounce(1);
    // feedBTrigger.toggleOnTrue(Commands.runOnce(() -> shooter.startFeed()));
  }

  public void createIntake() {
    robotIntake = new Intake();

    // Bottom Feed Trigger
    Trigger bottomFeedTrigger = controller.leftTrigger();
    bottomFeedTrigger.onTrue(robotIntake.startIntake()).onFalse(robotIntake.stopIntake());

    // Hopper Feed trigger
    Trigger hopperTrigger = controller.b();
    hopperTrigger.onTrue(robotIntake.startHopper()).onFalse(robotIntake.stopHopper());

    Trigger turnTrigger = controller.a();
    turnTrigger.onTrue(robotIntake.startHopper());
  }
}
