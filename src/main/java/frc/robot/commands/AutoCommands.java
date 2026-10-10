package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
// import frc.robot.Constants.FieldConstants;

public class AutoCommands {
  // Needs explanation
  private AutoCommands() {}

  // Makes robot orient towards hub and set shooter speed appropriately (togglable)
  public static Command HubCentric(Drive drivesys) {
    Pose2d RobotPose = drivesys.getPose();
    Translation2d NewRotation =
        RobotPose.getTranslation().rotateAround(new Translation2d(), RobotPose.getRotation());
    // Shuffleboard.getTab("Thing").add("NewTranslation", NewRotation);
    System.out.println(NewRotation);
    return Commands.run(() -> {});
  }

  /*
  public static Translation2d getHubTranslation2d() {
    Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isEmpty()) {
      return null;
    }

    return (alliance.get() == Alliance.Blue
        ? FieldConstants.blueHubCenterPose
        : FieldConstants.redHubCenterPose);
  }
  */
}
