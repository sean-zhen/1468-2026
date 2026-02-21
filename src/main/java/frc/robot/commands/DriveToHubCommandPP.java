package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;

public class DriveToHubCommandPP extends Command {

  private double endPtX, endPtY, endPtHoloRotation;

  private final Drive m_drive;

  public DriveToHubCommandPP(Drive drive) {
    m_drive = drive;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {

    // The rotation component in these poses represents the direction of travel
    Pose2d startPt = m_drive.getPose();
    double startX = m_drive.getPose().getX();
    Pose2d endPt;
    Pose2d interPt;
    List<Waypoint> wayPoints;

    if (DriverStation.getAlliance().isPresent()
        && (DriverStation.getAlliance().get() == Alliance.Red)
        && startX > 13.0) {
      //  Real Coordinates
      endPtX = 13.0;
      endPtY = 4.6;
      endPtHoloRotation = 180.0;
      endPt = new Pose2d(endPtX, endPtY, Rotation2d.fromDegrees(endPtHoloRotation));
      double interPtX = 13.3;
      interPt = new Pose2d(interPtX, endPtY, Rotation2d.fromDegrees(endPtHoloRotation));
    } else if (DriverStation.getAlliance().isPresent()
        && (DriverStation.getAlliance().get() == Alliance.Blue)
        && (startX < 3.5)) {
      endPtX = 3.5;
      endPtY = 3.6;
      endPtHoloRotation = 0.0;
      endPt = new Pose2d(endPtX, endPtY, Rotation2d.fromDegrees(endPtHoloRotation));
      double interPtX = 3.2;
      interPt = new Pose2d(interPtX, endPtY, Rotation2d.fromDegrees(endPtHoloRotation));
    } else {
      interPt = startPt;
      endPt = startPt;
    }

    wayPoints = PathPlannerPath.waypointsFromPoses(startPt, interPt, endPt);
    PathPlannerPath path =
        new PathPlannerPath(
            wayPoints,
            new PathConstraints(2.0, 2.0, Units.degreesToRadians(540), Units.degreesToRadians(720)),
            null,
            new GoalEndState(0.0, Rotation2d.fromDegrees(endPtHoloRotation)));

    // The first parameter is the position along the path (0.0 to 1.0)
    List<RotationTarget> rotationTargets = new ArrayList<>();
    rotationTargets.add(new RotationTarget(0.5, Rotation2d.fromDegrees(0.0)));
    // Prevent this path from being flipped on the red alliance, since the given
    // positions are already correct
    path.preventFlipping = true;

    AutoBuilder.followPath(path).schedule();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Shuffleboard.getTab("Drive")
        .add("DriveToProcessor status", "NOT ACTIVE")
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(0, 0)
        .withSize(3, 1);
    m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
