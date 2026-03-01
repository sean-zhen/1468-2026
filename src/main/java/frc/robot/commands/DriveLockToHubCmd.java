package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Shooter;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

public class DriveLockToHubCmd {

  // Define Hub coordinates (adjust to your actual field constants)
  private static final Translation2d BLUE_HUB = Shooter.BLUE_HUB_POS;
  private static final Translation2d RED_HUB = Shooter.RED_HUB_POS;

  public static Command create(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {

    return DriveCommands.joystickDriveAtAngle(
        drive,
        xSupplier,
        ySupplier,
        () -> {
          // 1. Get the current robot position from vision-fused odometry
          Translation2d robotPos = drive.getPose().getTranslation();

          // 2. Determine which Hub to look at
          boolean isRed =
              DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                  == DriverStation.Alliance.Red;
          Translation2d targetHub = isRed ? RED_HUB : BLUE_HUB;

          // 3. Calculate the angle from robot to Hub
          Translation2d diff = targetHub.minus(robotPos);
          // return new Rotation2d(Math.atan2(diff.getY(), diff.getX()));

          return new Rotation2d(Math.atan2(diff.getY(), diff.getX()))
              .plus(Rotation2d.fromDegrees(90));
        });
  }
}
