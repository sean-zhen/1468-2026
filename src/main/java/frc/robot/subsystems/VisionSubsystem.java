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
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import java.util.*;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class VisionSubsystem extends SubsystemBase {

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
  private boolean hasEverHadVision = false;
  private int lastFusedTagCount = 0;
  private double lastPoseResetTime = 0;
  private double lastPoseUpdateTime = 0;

  // --- Cached NT publishers: top-level ---
  private final BooleanPublisher healthyPub;
  private final IntegerPublisher rejectCountPub;
  private final BooleanPublisher initialResetPub;
  private final BooleanPublisher resetTriggeredPub;
  private final BooleanPublisher poseResetPub;
  private final BooleanPublisher poseUpdatedPub;

  // --- Cached NT publishers: per-camera ---
  private final BooleanPublisher[] camConnectedPub = new BooleanPublisher[4];
  private final IntegerPublisher[] camResultCountPub = new IntegerPublisher[4];
  private final BooleanPublisher[] camHasTargetsPub = new BooleanPublisher[4];
  private final IntegerPublisher[] camTargetsSeenPub = new IntegerPublisher[4];
  private final BooleanPublisher[] camHasMultiTagPub = new BooleanPublisher[4];
  private final BooleanPublisher[] camMultiTagPosePub = new BooleanPublisher[4];
  private final DoublePublisher[] camXPub = new DoublePublisher[4];
  private final DoublePublisher[] camYPub = new DoublePublisher[4];
  private final DoublePublisher[] camThetaPub = new DoublePublisher[4];
  private final IntegerPublisher[] camTagCountPub = new IntegerPublisher[4];
  private final DoublePublisher[] camScorePub = new DoublePublisher[4];
  private final DoublePublisher[] camMahalanobisPub = new DoublePublisher[4];

  // --- Cached NT publishers: fused ---
  private final DoublePublisher fusedXPub;
  private final DoublePublisher fusedYPub;
  private final DoublePublisher fusedThetaPub;
  private final IntegerPublisher fusedTagCountPub;
  private final IntegerPublisher fusedUsedCamerasPub;

  // --- Cached NT publishers: diagnostics ---
  private final IntegerPublisher diagCollectedPub;
  private final IntegerPublisher diagAfterGatePub;
  private final IntegerPublisher diagAfterAgreementPub;
  private final BooleanPublisher diagHasEverVisionPub;

  private static class Measurement {
    Pose2d pose;
    double score;
    int tagCount;
    double mahalanobis;
    int camIndex;
    double timestamp;
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

  public VisionSubsystem(Drive drive) {
    this.drive = drive;
    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    for (int i = 0; i < 4; i++) {
      cameras[i] = new PhotonCamera(CAM_NAMES[i]);
      estimators[i] = new PhotonPoseEstimator(fieldLayout, CAMERA_TRANSFORMS[i]);
    }

    // Cache top-level publishers
    healthyPub = visionTable.getBooleanTopic("Healthy").publish();
    rejectCountPub = visionTable.getIntegerTopic("RejectCount").publish();
    initialResetPub = visionTable.getBooleanTopic("InitialReset").publish();
    resetTriggeredPub = visionTable.getBooleanTopic("ResetTriggered").publish();
    poseResetPub = visionTable.getBooleanTopic("PoseReset").publish();
    poseUpdatedPub = visionTable.getBooleanTopic("PoseUpdated").publish();

    // Cache per-camera publishers
    for (int i = 0; i < 4; i++) {
      NetworkTable camTable = visionTable.getSubTable("Cam" + (i + 1));
      NetworkTable dbg = camTable.getSubTable("Debug");
      camConnectedPub[i] = dbg.getBooleanTopic("Connected").publish();
      camResultCountPub[i] = dbg.getIntegerTopic("ResultCount").publish();
      camHasTargetsPub[i] = dbg.getBooleanTopic("HasTargets").publish();
      camTargetsSeenPub[i] = dbg.getIntegerTopic("TargetsSeen").publish();
      camHasMultiTagPub[i] = dbg.getBooleanTopic("HasMultiTagResult").publish();
      camMultiTagPosePub[i] = dbg.getBooleanTopic("MultiTagPosePresent").publish();
      camXPub[i] = camTable.getDoubleTopic("X").publish();
      camYPub[i] = camTable.getDoubleTopic("Y").publish();
      camThetaPub[i] = camTable.getDoubleTopic("ThetaDeg").publish();
      camTagCountPub[i] = camTable.getIntegerTopic("TagCount").publish();
      camScorePub[i] = camTable.getDoubleTopic("Score").publish();
      camMahalanobisPub[i] = camTable.getDoubleTopic("Mahalanobis").publish();
    }

    // Cache fused publishers
    NetworkTable fusedTable = visionTable.getSubTable("Fused");
    fusedXPub = fusedTable.getDoubleTopic("X").publish();
    fusedYPub = fusedTable.getDoubleTopic("Y").publish();
    fusedThetaPub = fusedTable.getDoubleTopic("ThetaDeg").publish();
    fusedTagCountPub = fusedTable.getIntegerTopic("TagCount").publish();
    fusedUsedCamerasPub = fusedTable.getIntegerTopic("UsedCameras").publish();

    // Cache diagnostics publishers
    NetworkTable diagTable = visionTable.getSubTable("Diagnostics");
    diagCollectedPub = diagTable.getIntegerTopic("CollectedCount").publish();
    diagAfterGatePub = diagTable.getIntegerTopic("AfterMahalanobisGate").publish();
    diagAfterAgreementPub = diagTable.getIntegerTopic("AfterAgreementFilter").publish();
    diagHasEverVisionPub = diagTable.getBooleanTopic("HasEverHadVision").publish();
  }

  @Override
  public void periodic() {
    double velocity =
        Math.hypot(
            drive.getActualChassisSpeeds().vxMetersPerSecond,
            drive.getActualChassisSpeeds().vyMetersPerSecond);

    List<Measurement> validMeasurements = collectMeasurements(velocity);

    // If we've never had a vision update, skip the Mahalanobis gate entirely
    // and hard-reset the pose from the first good multi-tag result.
    if (!hasEverHadVision && !validMeasurements.isEmpty()) {
      Measurement best = validMeasurements.get(0);
      for (Measurement m : validMeasurements) {
        if (m.score > best.score) best = m;
      }
      drive.resetPose(best.pose);
      hasEverHadVision = true;
      lastVisionTimestamp = Timer.getFPGATimestamp();
      lastResetTime = Timer.getFPGATimestamp();
      lastPoseResetTime = Timer.getFPGATimestamp();
      initialResetPub.set(true);
      poseResetPub.set(true);
      SmartDashboard.putBoolean("Vision/Pose Reset", true);
      logFusion(best, validMeasurements);
      logDiagnostics(validMeasurements.size(), validMeasurements.size(), validMeasurements.size());
      return;
    }

    List<Measurement> gated = mahalanobisGate(validMeasurements);
    List<Measurement> agreeing = agreementFilter(gated);

    logDiagnostics(validMeasurements.size(), gated.size(), agreeing.size());

    if (!agreeing.isEmpty()) {
      Measurement fused = fuse(agreeing, velocity);
      applyToEstimator(fused, velocity);
      maybeHardReset(fused, velocity);
      lastVisionTimestamp = Timer.getFPGATimestamp();
      hasEverHadVision = true;
      logFusion(fused, agreeing);
    } else {
      logNoVision();
    }

    healthyPub.set((Timer.getFPGATimestamp() - lastVisionTimestamp) < 0.5);
    rejectCountPub.set(rejectCount);

    // Update state-based indicators
    double currentTime = Timer.getFPGATimestamp();
    
    // Pose Reset: Green for 0.5s after reset, then red
    boolean poseResetActive = (currentTime - lastPoseResetTime) < 0.5;
    poseResetPub.set(poseResetActive);
    SmartDashboard.putBoolean("Vision/Pose Reset", poseResetActive);
    
    // Pose Updated: Green while actively updating, red if no update for 0.5s
    boolean poseUpdateActive = (currentTime - lastPoseUpdateTime) < 0.5;
    poseUpdatedPub.set(poseUpdateActive);
    SmartDashboard.putBoolean("Vision/Pose Updated", poseUpdateActive);
  }

  private List<Measurement> collectMeasurements(double velocity) {
    List<Measurement> list = new ArrayList<>();
    for (int i = 0; i < 4; i++) {
      var results = cameras[i].getAllUnreadResults();
      camConnectedPub[i].set(cameras[i].isConnected());
      camResultCountPub[i].set(results.size());

      for (var result : results) {
        camHasTargetsPub[i].set(result.hasTargets());
        if (!result.hasTargets()) continue;

        camTargetsSeenPub[i].set(result.getTargets().size());
        camHasMultiTagPub[i].set(result.getMultiTagResult().isPresent());

        Optional<EstimatedRobotPose> poseOpt = estimators[i].estimateCoprocMultiTagPose(result);
        camMultiTagPosePub[i].set(poseOpt.isPresent());
        if (poseOpt.isEmpty()) continue;

        int tagCount = result.getTargets().size();
        if (tagCount < 2) continue; // multi-tag only

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
        m.timestamp = poseOpt.get().timestampSeconds;
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

      camMahalanobisPub[m.camIndex].set(d2);
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

      // Accept the fallback only if it sees at least 2 tags
      if (best.tagCount >= 2) {
        agreeing.add(best);
      }
    }

    return agreeing;
  }

  private Measurement fuse(List<Measurement> list, double velocity) {
    double totalWeight = 0, x = 0, y = 0, theta = 0;
    int totalTags = 0;
    double bestTimestamp = 0;
    double bestScore = -1;
    for (Measurement m : list) {
      totalWeight += m.score;
      x += m.pose.getX() * m.score;
      y += m.pose.getY() * m.score;
      theta += m.pose.getRotation().getRadians() * m.score;
      totalTags += m.tagCount;
      if (m.score > bestScore) {
        bestScore = m.score;
        bestTimestamp = m.timestamp;
      }
    }
    x /= totalWeight;
    y /= totalWeight;
    theta /= totalWeight;

    Measurement fused = new Measurement();
    fused.pose = new Pose2d(x, y, new Rotation2d(theta));
    fused.score = totalWeight;
    fused.tagCount = totalTags;
    fused.timestamp = bestTimestamp;
    return fused;
  }

  private void applyToEstimator(Measurement fused, double velocity) {
    Vector<N3> stdDevs =
        VecBuilder.fill(
            0.05 * (1 + velocity / MAX_SPEED),
            0.05 * (1 + velocity / MAX_SPEED),
            0.1 * (1 + velocity / MAX_SPEED));
    drive.addVisionMeasurement(fused.pose, fused.timestamp, stdDevs, "FUSED");
    lastPoseUpdateTime = Timer.getFPGATimestamp();
    poseUpdatedPub.set(true);
    SmartDashboard.putBoolean("Vision/Pose Updated", true);
  }

  private void maybeHardReset(Measurement fused, double velocity) {
    if (velocity > RESET_SPEED_THRESHOLD) return;
    if (fused.tagCount < 3) return;
    if (Timer.getFPGATimestamp() - lastResetTime < RESET_STABILITY_TIME) return;

    drive.resetPose(fused.pose);
    lastResetTime = Timer.getFPGATimestamp();
    lastPoseResetTime = Timer.getFPGATimestamp();
    resetTriggeredPub.set(true);
    poseResetPub.set(true);
    SmartDashboard.putBoolean("Vision/Pose Reset", true);
  }

  private void logCamera(int idx, Pose2d pose, int tagCount, double score) {
    camXPub[idx].set(pose.getX());
    camYPub[idx].set(pose.getY());
    camThetaPub[idx].set(pose.getRotation().getDegrees());
    camTagCountPub[idx].set(tagCount);
    camScorePub[idx].set(score);
  }

  private void logFusion(Measurement fused, List<Measurement> used) {
    fusedXPub.set(fused.pose.getX());
    fusedYPub.set(fused.pose.getY());
    fusedThetaPub.set(fused.pose.getRotation().getDegrees());
    fusedTagCountPub.set(fused.tagCount);
    fusedUsedCamerasPub.set(used.size());
    lastFusedTagCount = fused.tagCount;
  }

  private void logNoVision() {
    fusedXPub.set(0);
    fusedYPub.set(0);
    fusedThetaPub.set(0);
    fusedTagCountPub.set(0);
    fusedUsedCamerasPub.set(0);
    lastFusedTagCount = 0;
  }

  private void logDiagnostics(int collected, int afterGate, int afterAgreement) {
    diagCollectedPub.set(collected);
    diagAfterGatePub.set(afterGate);
    diagAfterAgreementPub.set(afterAgreement);
    diagHasEverVisionPub.set(hasEverHadVision);
  }

  public Optional<Translation2d> getTagPosition(int tagId) {
    Optional<Pose3d> tagPose3d = fieldLayout.getTagPose(tagId);
    return tagPose3d.map(p -> p.toPose2d().getTranslation());
  }

  public AprilTagFieldLayout getFieldLayout() {
    return fieldLayout;
  }

  public int getFusedTagCount() {
    return lastFusedTagCount;
  }
}
