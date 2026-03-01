package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import java.util.*;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class backupVS extends SubsystemBase {

  private static final String[] CAM_NAMES = {"Camera-1", "Camera-2", "Camera-3", "Camera-4"};
  private static final double MAX_SPEED = 4.5;
  private static final double AGREEMENT_POS_TOL = 0.075;
  private static final double AGREEMENT_ROT_TOL_DEG = 10.0;
  private static final double RESET_SPEED_THRESHOLD = 0.4;
  private static final double RESET_STABILITY_TIME = 0.25;
  private static final double MAHALANOBIS_THRESHOLD = 7.8;

  private final Drive drive;
  private final AprilTagFieldLayout fieldLayout;
  private final PhotonCamera[] cameras = new PhotonCamera[4];
  private final PhotonPoseEstimator[] estimators = new PhotonPoseEstimator[4];
  private final NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("Vision");

  private double lastResetTime = 0;
  private double lastVisionTimestamp = 0;
  private int rejectCount = 0;

  private static class Measurement {
    Pose2d pose;
    double score;
    int tagCount;
    double mahalanobis;
    int camIndex;
  }

  private static final Transform3d[] CAMERA_TRANSFORMS = {
    new Transform3d(
        new Translation3d(
            Units.inchesToMeters(-7.857),
            Units.inchesToMeters(-12.293),
            Units.inchesToMeters(8.181)),
        new Rotation3d(0, Units.degreesToRadians(-18), Units.degreesToRadians(290))), // Cam 1
    new Transform3d(
        new Translation3d(
            Units.inchesToMeters(-11.946),
            Units.inchesToMeters(-10.063),
            Units.inchesToMeters(7.799)),
        new Rotation3d(0, Units.degreesToRadians(-28), Units.degreesToRadians(140))), // Cam 2
    new Transform3d(
        new Translation3d(
            Units.inchesToMeters(-11.946),
            Units.inchesToMeters(10.063),
            Units.inchesToMeters(7.799)),
        new Rotation3d(0, Units.degreesToRadians(-28), Units.degreesToRadians(220))), // Cam 3
    new Transform3d(
        new Translation3d(
            Units.inchesToMeters(-7.857),
            Units.inchesToMeters(12.293),
            Units.inchesToMeters(8.181)),
        new Rotation3d(0, Units.degreesToRadians(-18), Units.degreesToRadians(70))), // Cam 4
  };

  public backupVS(Drive drive) {
    this.drive = drive;
    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    for (int i = 0; i < 4; i++) {
      cameras[i] = new PhotonCamera(CAM_NAMES[i]);
      estimators[i] = new PhotonPoseEstimator(fieldLayout, CAMERA_TRANSFORMS[i]);
    }
  }

  @Override
  public void periodic() {
    double velocity =
        Math.hypot(
            drive.getActualChassisSpeeds().vxMetersPerSecond,
            drive.getActualChassisSpeeds().vyMetersPerSecond);

    List<Measurement> validMeasurements = collectMeasurements(velocity);
    List<Measurement> gated = mahalanobisGate(validMeasurements);
    List<Measurement> agreeing = agreementFilter(gated);

    if (!agreeing.isEmpty()) {
      Measurement fused = fuse(agreeing, velocity);
      applyToEstimator(fused, velocity);
      maybeHardReset(fused, velocity);
      lastVisionTimestamp = Timer.getFPGATimestamp();
      logFusion(fused, agreeing);
    } else {
      logNoVision();
    }

    visionTable
        .getEntry("Healthy")
        .setBoolean((Timer.getFPGATimestamp() - lastVisionTimestamp) < 0.5);
    visionTable.getEntry("RejectCount").setInteger(rejectCount);
  }

  private List<Measurement> collectMeasurements(double velocity) {
    List<Measurement> list = new ArrayList<>();
    for (int i = 0; i < 4; i++) {
      var results = cameras[i].getAllUnreadResults();
      for (var result : results) {
        if (!result.hasTargets()) continue;
        Optional<EstimatedRobotPose> poseOpt = estimators[i].estimateCoprocMultiTagPose(result);
        if (poseOpt.isEmpty()) continue;

        int tagCount = result.getTargets().size();
        if (tagCount < 2) continue; // multi-tag filter

        Pose2d pose = poseOpt.get().estimatedPose.toPose2d();
        double avgDist =
            result.getTargets().stream()
                .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
                .average()
                .orElse(4.0);

        Measurement m = new Measurement();
        m.pose = pose;
        m.score = computeScore(tagCount, avgDist, velocity);
        m.tagCount = tagCount;
        m.camIndex = i;
        list.add(m);

        logCamera(i, pose, tagCount, m.score);
      }
    }
    return list;
  }

  private double computeScore(int tagCount, double avgDist, double velocity) {
    double tagWeight = tagCount * tagCount;
    double distanceWeight = 1.0 / (avgDist * avgDist);
    double velocityWeight = 1.0 / (1.0 + Math.pow(velocity / MAX_SPEED, 2));
    return tagWeight * distanceWeight * velocityWeight;
  }

  private List<Measurement> mahalanobisGate(List<Measurement> measurements) {
    List<Measurement> accepted = new ArrayList<>();
    Pose2d current = drive.getPose();

    for (Measurement m : measurements) {
      double dx = m.pose.getX() - current.getX();
      double dy = m.pose.getY() - current.getY();
      double dtheta = m.pose.getRotation().minus(current.getRotation()).getRadians();
      double variance = 0.5;
      double d2 = (dx * dx + dy * dy + dtheta * dtheta) / variance;

      m.mahalanobis = d2;
      if (d2 < MAHALANOBIS_THRESHOLD) accepted.add(m);
      else rejectCount++;

      visionTable.getSubTable("Cam" + (m.camIndex + 1)).getEntry("Mahalanobis").setDouble(d2);
    }
    return accepted;
  }

  private List<Measurement> agreementFilter(List<Measurement> list) {
    // If we only have one measurement, we can't check for agreement,
    // so we just return it.
    if (list.size() <= 1) return list;

    List<Measurement> agreeing = new ArrayList<>();

    // 1. ATTEMPT CONSENSUS: Look for two cameras that see the same thing
    for (Measurement m : list) {
      for (Measurement other : list) {
        if (m == other) continue;

        double posDiff = m.pose.getTranslation().getDistance(other.pose.getTranslation());
        double rotDiff =
            Math.abs(m.pose.getRotation().minus(other.pose.getRotation()).getDegrees());

        if (posDiff < AGREEMENT_POS_TOL && rotDiff < AGREEMENT_ROT_TOL_DEG) {
          agreeing.add(m);
          break; // This camera is "verified" by another
        }
      }
    }

    // 2. FALLBACK: If no cameras agreed, pick the single most mathematically reliable one
    if (agreeing.isEmpty() && !list.isEmpty()) {
      Measurement best = list.get(0);
      for (Measurement m : list) {
        if (m.score > best.score) {
          best = m;
        }
      }

      // OPTIONAL: Only accept the fallback if it's actually a "good" look
      // For example, only if it sees at least 2 tags.
      if (best.tagCount >= 2) {
        agreeing.add(best);
      }
    }

    return agreeing;
  }

  private Measurement fuse(List<Measurement> list, double velocity) {
    double totalWeight = 0, x = 0, y = 0, theta = 0;
    int totalTags = 0;
    for (Measurement m : list) {
      totalWeight += m.score;
      x += m.pose.getX() * m.score;
      y += m.pose.getY() * m.score;
      theta += m.pose.getRotation().getRadians() * m.score;
      totalTags += m.tagCount;
    }
    x /= totalWeight;
    y /= totalWeight;
    theta /= totalWeight;

    Measurement fused = new Measurement();
    fused.pose = new Pose2d(x, y, new Rotation2d(theta));
    fused.score = totalWeight;
    fused.tagCount = totalTags;
    return fused;
  }

  private void applyToEstimator(Measurement fused, double velocity) {
    Vector<N3> stdDevs =
        VecBuilder.fill(
            0.05 * (1 + velocity / MAX_SPEED),
            0.05 * (1 + velocity / MAX_SPEED),
            0.1 * (1 + velocity / MAX_SPEED));
    drive.addVisionMeasurement(fused.pose, Timer.getFPGATimestamp(), stdDevs, "FUSED");
  }

  private void maybeHardReset(Measurement fused, double velocity) {
    if (velocity > RESET_SPEED_THRESHOLD) return;
    if (fused.tagCount < 3) return;
    if (Timer.getFPGATimestamp() - lastResetTime < RESET_STABILITY_TIME) return;

    drive.resetPose(fused.pose);
    lastResetTime = Timer.getFPGATimestamp();
    visionTable.getEntry("ResetTriggered").setBoolean(true);
  }

  private void logCamera(int idx, Pose2d pose, int tagCount, double score) {
    NetworkTable camTable = visionTable.getSubTable("Cam" + (idx + 1));
    camTable.getEntry("X").setDouble(pose.getX());
    camTable.getEntry("Y").setDouble(pose.getY());
    camTable.getEntry("ThetaDeg").setDouble(pose.getRotation().getDegrees());
    camTable.getEntry("TagCount").setInteger(tagCount);
    camTable.getEntry("Score").setDouble(score);
  }

  private void logFusion(Measurement fused, List<Measurement> used) {
    NetworkTable fusedTable = visionTable.getSubTable("Fused");
    fusedTable.getEntry("X").setDouble(fused.pose.getX());
    fusedTable.getEntry("Y").setDouble(fused.pose.getY());
    fusedTable.getEntry("ThetaDeg").setDouble(fused.pose.getRotation().getDegrees());
    fusedTable.getEntry("TagCount").setInteger(fused.tagCount);
    fusedTable.getEntry("UsedCameras").setInteger(used.size());
  }

  private void logNoVision() {
    NetworkTable fusedTable = visionTable.getSubTable("Fused");
    fusedTable.getEntry("X").setDouble(0);
    fusedTable.getEntry("Y").setDouble(0);
    fusedTable.getEntry("ThetaDeg").setDouble(0);
    fusedTable.getEntry("TagCount").setInteger(0);
    fusedTable.getEntry("UsedCameras").setInteger(0);
  }

  public Optional<Translation2d> getTagPosition(int tagId) {
    Optional<Pose3d> tagPose3d = fieldLayout.getTagPose(tagId);
    return tagPose3d.map(p -> p.toPose2d().getTranslation());
  }

  public AprilTagFieldLayout getFieldLayout() {
    return fieldLayout;
  }

  public int getFusedTagCount() {
    NetworkTable fusedTable = visionTable.getSubTable("Fused");
    return (int) fusedTable.getEntry("TagCount").getDouble(0.0);
  }
}
