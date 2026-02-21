// HAVE TO BIND TO BUTTON
// THIS IS AI GENERATED. WE REALLY NEED TO TEST THIS TO SEE HOW IT WORKDS WITH THE TURRET

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Shooter;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drive.Drive;
import java.util.List;
import java.util.Optional;

/**
 * Command that tracks alliance-specific AprilTags with the turret. Automatically selects hub tags
 * based on alliance color and aims at the closest one.
 */
public class TurretAprilTagTracking extends Command {
  private final ShooterSubsystem shooter;
  private final VisionSubsystem vision;
  private final Drive drive;

  // Alliance-specific hub tags
  private static final List<Integer> RED_ALLIANCE_TAGS = List.of(2, 3, 4, 5, 8, 9, 10, 11);
  private static final List<Integer> BLUE_ALLIANCE_TAGS = List.of(18, 19, 20, 21, 24, 25, 26, 27);

  private final PIDController turretController;

  /**
   * Creates a command to track alliance-specific AprilTags with the turret.
   *
   * @param shooter The shooter subsystem (contains turret)
   * @param vision The vision subsystem
   * @param drive The drive subsystem (for robot pose)
   */
  public TurretAprilTagTracking(ShooterSubsystem shooter, VisionSubsystem vision, Drive drive) {
    this.shooter = shooter;
    this.vision = vision;
    this.drive = drive;

    addRequirements(shooter);

    // Configure turret PID controller
    turretController = new PIDController(Shooter.TURRET_kP, Shooter.TURRET_kI, Shooter.TURRET_kD);
    turretController.enableContinuousInput(-Math.PI, Math.PI);
    turretController.setTolerance(Units.degreesToRadians(Shooter.TURRET_TRACKING_TOLERANCE_DEG));
  }

  @Override
  public void initialize() {
    turretController.reset();

    var alliance = DriverStation.getAlliance();
    String allianceColor = alliance.isPresent() ? alliance.get().toString() : "UNKNOWN";
    var shooterTab = Shuffleboard.getTab("Shooter");
    shooterTab
        .add("TurretTracking/Alliance", allianceColor)
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(0, 5)
        .withSize(2, 1);
    shooterTab
        .add("TurretTracking/Status", "TRACKING")
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(2, 5)
        .withSize(2, 1);
  }

  @Override
  public void execute() {
    // Get alliance and determine target tags
    var alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      Shuffleboard.getTab("Shooter")
          .add("TurretTracking/Status", "NO ALLIANCE DATA")
          .withWidget(BuiltInWidgets.kTextView)
          .withPosition(2, 5)
          .withSize(2, 1);
      shooter.setTurretVelocity(0.0);
      return;
    }

    List<Integer> targetTags =
        alliance.get() == Alliance.Red ? RED_ALLIANCE_TAGS : BLUE_ALLIANCE_TAGS;

    // Get robot pose from odometry
    Pose2d robotPose = drive.getPose();
    Translation2d robotPosition = robotPose.getTranslation();
    Rotation2d robotHeading = robotPose.getRotation();

    // Find the closest valid tag
    Optional<Translation2d> closestTagPosition = Optional.empty();
    int closestTagId = -1;
    double closestDistance = Double.MAX_VALUE;

    for (int tagId : targetTags) {
      Optional<Translation2d> tagPosition = getTagPosition(tagId);
      if (tagPosition.isPresent()) {
        double distance = robotPosition.getDistance(tagPosition.get());
        if (distance < closestDistance) {
          closestDistance = distance;
          closestTagPosition = tagPosition;
          closestTagId = tagId;
        }
      }
    }

    if (closestTagPosition.isEmpty()) {
      Shuffleboard.getTab("Shooter")
          .add("TurretTracking/Status", "NO TAGS FOUND")
          .withWidget(BuiltInWidgets.kTextView)
          .withPosition(2, 5)
          .withSize(2, 1);
      shooter.setTurretVelocity(0.0);
      return;
    }

    // Calculate angle from robot to tag (field-relative)
    Translation2d robotToTag = closestTagPosition.get().minus(robotPosition);
    Rotation2d angleToTag = robotToTag.getAngle();

    // Calculate turret angle relative to robot
    // Turret angle = (angle to tag) - (robot heading)
    Rotation2d turretTargetAngle = angleToTag.minus(robotHeading);

    // Use PID to calculate turret velocity
    double currentTurretAngle = shooter.getTurretPosition() * 2 * Math.PI; // Convert to radians
    double turretVelocity =
        turretController.calculate(currentTurretAngle, turretTargetAngle.getRadians());

    // Clamp velocity to max speed
    turretVelocity =
        Math.max(
            -Shooter.TURRET_MAX_VELOCITY_RPS,
            Math.min(Shooter.TURRET_MAX_VELOCITY_RPS, turretVelocity));

    // Apply velocity to turret
    shooter.setTurretVelocity(turretVelocity);

    // ── Logging on the Shooter tab ────────────────────────────────────────────
    var shooterTab = Shuffleboard.getTab("Shooter");
    shooterTab
        .add("TurretTracking/Status", "TRACKING")
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(2, 5)
        .withSize(2, 1);
    shooterTab
        .add("TurretTracking/TargetTagID", closestTagId)
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(4, 5)
        .withSize(2, 1);
    shooterTab
        .add("TurretTracking/DistanceToTag", closestDistance)
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(6, 5)
        .withSize(2, 1);
    shooterTab
        .add("TurretTracking/RobotX", robotPosition.getX())
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(0, 6)
        .withSize(2, 1);
    shooterTab
        .add("TurretTracking/RobotY", robotPosition.getY())
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(2, 6)
        .withSize(2, 1);
    shooterTab
        .add("TurretTracking/RobotHeading", robotHeading.getDegrees())
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(4, 6)
        .withSize(2, 1);
    shooterTab
        .add("TurretTracking/TagX", closestTagPosition.get().getX())
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(0, 7)
        .withSize(2, 1);
    shooterTab
        .add("TurretTracking/TagY", closestTagPosition.get().getY())
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(2, 7)
        .withSize(2, 1);
    shooterTab
        .add("TurretTracking/AngleToTag", angleToTag.getDegrees())
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(4, 7)
        .withSize(2, 1);
    shooterTab
        .add("TurretTracking/TurretTargetAngle", turretTargetAngle.getDegrees())
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(0, 8)
        .withSize(2, 1);
    shooterTab
        .add(
            "TurretTracking/TurretCurrentAngle",
            Units.radiansToDegrees(currentTurretAngle))
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(2, 8)
        .withSize(2, 1);
    shooterTab
        .add("TurretTracking/TurretVelocity", turretVelocity)
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(4, 8)
        .withSize(2, 1);
    shooterTab
        .add("TurretTracking/AtTarget", turretController.atSetpoint())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(6, 8)
        .withSize(2, 1);
  }

  /**
   * Get tag position from vision field layout.
   *
   * @param tagId The AprilTag ID
   * @return Optional containing tag position, or empty if not found
   */
  private Optional<Translation2d> getTagPosition(int tagId) {
    var tagPose = vision.getFieldLayout().getTagPose(tagId);
    if (tagPose.isPresent()) {
      return Optional.of(tagPose.get().toPose2d().getTranslation());
    }
    return Optional.empty();
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setTurretVelocity(0.0);
    Shuffleboard.getTab("Shooter")
        .add("TurretTracking/Status", interrupted ? "INTERRUPTED" : "FINISHED")
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(2, 5)
        .withSize(2, 1);
  }

  @Override
  public boolean isFinished() {
    return false; // Runs until interrupted
  }
}
