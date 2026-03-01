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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.util.Collections;
import java.util.List;
import java.util.Set;

public class DriveToDepotCommandPP {

  public static Command create(Drive drive) {

    return Commands.defer(
        () -> {
          SmartDashboard.putString("DriveToDepot status", "CALCULATING");

          Pose2d startPt = drive.getPose();
          double startX = startPt.getX();

          double endPtX, endPtY, endPtHoloRotation;
          Pose2d endPt, interPt;

          // RED ALLIANCE LOGIC
          if (DriverStation.getAlliance().isPresent()
              && (DriverStation.getAlliance().get() == Alliance.Red)
              && startX > 13.0) { // Make sure this startX threshold makes sense for your depot zone

            // TODO: Update these to your actual Red Depot coordinates
            endPtX = 16.06;
            endPtY = 7.4;
            // 180 degrees points away from the Red wall, out towards the field
            endPtHoloRotation = -90.0;

            endPt = new Pose2d(endPtX, endPtY, Rotation2d.fromDegrees(endPtHoloRotation));

            // Intermediate point slightly closer to the wall
            double interPtX = endPtX - 1.0;
            interPt = new Pose2d(interPtX, endPtY, Rotation2d.fromDegrees(endPtHoloRotation));

            // BLUE ALLIANCE LOGIC
          } else if (DriverStation.getAlliance().isPresent()
              && (DriverStation.getAlliance().get() == Alliance.Blue)
              && (startX
                  < 3.5)) { // Make sure this startX threshold makes sense for your depot zone

            // TODO: Update these to your actual Blue Depot coordinates
            endPtX = 0.48;
            endPtY = 0.64;
            // 0 degrees points away from the Blue wall, out towards the field
            endPtHoloRotation = 90.0;

            endPt = new Pose2d(endPtX, endPtY, Rotation2d.fromDegrees(endPtHoloRotation));

            // Intermediate point slightly closer to the wall
            double interPtX = endPtX + 1.0;
            interPt = new Pose2d(interPtX, endPtY, Rotation2d.fromDegrees(endPtHoloRotation));

          } else {
            // Fallback: Stay put if zones don't match
            interPt = startPt;
            endPt = startPt;
            endPtHoloRotation = startPt.getRotation().getDegrees();
          }

          List<Waypoint> wayPoints = PathPlannerPath.waypointsFromPoses(startPt, interPt, endPt);

          // FIXED: Now uses the alliance-specific rotation target instead of a hardcoded 0.0
          List<RotationTarget> rotationTargets =
              List.of(new RotationTarget(1.0, Rotation2d.fromDegrees(endPtHoloRotation)));

          PathPlannerPath path =
              new PathPlannerPath(
                  wayPoints,
                  rotationTargets,
                  Collections.emptyList(),
                  Collections.emptyList(),
                  Collections.emptyList(),
                  new PathConstraints(
                      2.0, 2.0, Units.degreesToRadians(540), Units.degreesToRadians(720)),
                  null,
                  new GoalEndState(0.0, Rotation2d.fromDegrees(endPtHoloRotation)),
                  false);

          path.preventFlipping = true;

          SmartDashboard.putString("DriveToDepot status", "RUNNING");

          return AutoBuilder.followPath(path);
        },
        Set.of(drive));
  }
}
