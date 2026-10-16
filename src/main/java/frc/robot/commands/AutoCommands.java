package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.Constants;

public class AutoCommands {
  // Needs explanation
  private AutoCommands() {}

  // Makes robot orient towards hub and set shooter speed appropriately (togglable)
  public static Command HubCentric(Drive drivesys) {
    Pose2d currentPose = drivesys.getPose();

    Translation2d hubCenterPose = new Translation2d(10, 10);//ShooterUtil.getHubTranslation2d();

    System.out.println(NewRotation);
    return Commands.run(() -> {});
  }

  public void aimDrive(double xSpeed, double ySpeed) { 



      //end method if there is no alliance selected cuz this should only be used on the field
      if (hubCenterPose == null) {
        return;
      }
      
      double theta;

      double xDisplacement = (hubCenterPose.getX() - currentPose.getX()); 
      double yDisplacement = (hubCenterPose.getY() - currentPose.getY());

      theta = Math.toDegrees(Math.atan2(yDisplacement, xDisplacement)); 

      theta += 180;
      
      double rotOutput = aimDriveController.calculate(getRotation().getDegrees(), theta); 

      drive(new ChassisSpeeds(xSpeed, ySpeed, rotOutput), true);
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
