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

// 1. Remove 'extends Command'. This is now a factory class.
public class DriveToHubCommandPP {

  // 2. Change to a STATIC method that returns a Command
  public static Command create(Drive drive) {

    // 3. The Defer Block
    return Commands.defer(
        () -> {

          // --- EVERYTHING INSIDE HERE RUNS "JUST IN TIME" ---

          SmartDashboard.putString("DriveToHub status", "CALCULATING");

          Pose2d startPt = drive.getPose();
          double startX = startPt.getX();

          // Local variables (Do not use class fields)
          double endPtX, endPtY, endPtHoloRotation;
          Pose2d endPt, interPt;

          // Logic to determine Alliance/Target
          if (DriverStation.getAlliance().isPresent()
              && (DriverStation.getAlliance().get() == Alliance.Red)
              && startX > 13.0) {
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
            // Fallback: Stay put if zones don't match
            interPt = startPt;
            endPt = startPt;
            endPtHoloRotation = startPt.getRotation().getDegrees();
          }

          List<Waypoint> wayPoints = PathPlannerPath.waypointsFromPoses(startPt, interPt, endPt);

          // 4. Define Rotation Targets BEFORE creating the path
          // Note: Index 1.0 is the intermediate point (Start=0, Inter=1, End=2)
          List<RotationTarget> rotationTargets =
              List.of(new RotationTarget(1.0, Rotation2d.fromDegrees(0.0)));

          // 5. Use Full Constructor to inject RotationTargets
          PathPlannerPath path =
              new PathPlannerPath(
                  wayPoints,
                  rotationTargets, // <--- Targets go here
                  Collections.emptyList(), // PointTowardsZones
                  Collections.emptyList(), // ConstraintZones
                  Collections.emptyList(), // EventMarkers
                  new PathConstraints(
                      2.0, 2.0, Units.degreesToRadians(540), Units.degreesToRadians(720)),
                  null, // IdealStartingState
                  new GoalEndState(0.0, Rotation2d.fromDegrees(endPtHoloRotation)),
                  false // Reversed
                  );

          path.preventFlipping = true;

          SmartDashboard.putString("DriveToHub status", "RUNNING");

          // Return the specific command to follow this newly generated path
          return AutoBuilder.followPath(path);
        },
        Set.of(drive)); // 6. REQUIREMENTS DECLARED HERE
  }
}
