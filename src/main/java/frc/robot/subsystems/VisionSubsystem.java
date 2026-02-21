package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionSubsystem extends SubsystemBase {
  // Camera Names
  private static final String FRONT_NAME = "Camera_1";
  private static final String BACK_NAME = "Camera_2";
  // private static final String LEFT_NAME = "leftCamera";
  // private static final String RIGHT_NAME = "rightCamera";

  private final Field2d m_field = new Field2d();

  // Constants
  private static final double MAX_SINGLE_TAG_DIST = 3.0; // Meters
  private static final double MAX_RESET_VELOCITY = 0.5; // M/s
  private static final double VISION_TIMEOUT = 0.5; // Seconds

  // Confidence Matrices
  private static final Matrix<N3, N1> SINGLE_TAG_TELEOP = VecBuilder.fill(0.1, 0.1, 4.0);
  private static final Matrix<N3, N1> SINGLE_TAG_AUTO = VecBuilder.fill(4.0, 4.0, 8.0);
  private static final Matrix<N3, N1> MULTI_TAG = VecBuilder.fill(0.1, 0.1, 0.25);

  // private final PhotonCamera frontCam, backCam, leftCam, rightCam;
  private final PhotonCamera frontCam, backCam;
  // private final PhotonPoseEstimator frontEst, backEst, leftEst, rightEst;
  private final PhotonPoseEstimator frontEst, backEst;

  private final AprilTagFieldLayout fieldLayout;
  private final Drive drive;

  public static double cam1_x = Units.inchesToMeters(13);
  public static double cam1_y = Units.inchesToMeters(-12);
  public static double cam1_z = Units.inchesToMeters(15.0);
  public static double cam1_roll = 0;
  public static double cam1_pitch = 0;
  public static double cam1_yaw = 0;

  public static double cam2_x = Units.inchesToMeters(13);
  public static double cam2_y = Units.inchesToMeters(-6);
  public static double cam2_z = Units.inchesToMeters(15.0);
  public static double cam2_roll = 0;
  public static double cam2_pitch = 0;
  public static double cam2_yaw = 0;

  private double lastMeasurementTimestamp = 0;
  private int lastTagCount = 0;

  // Data Cache Class
  private static class CameraData {
    public PhotonPipelineResult result = new PhotonPipelineResult();
    public Optional<EstimatedRobotPose> pose = Optional.empty();
    public long loopTimeNanos = 0;
  }

  private final CameraData fData = new CameraData(),
      bData = new CameraData(),
      lData = new CameraData(),
      rData = new CameraData();

  // Shuffleboard entries
  private final ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");
  private final GenericEntry backPresentEntry;
  private final GenericEntry visionHealthyEntry;
  private final GenericEntry activeSourceEntry;
  private final GenericEntry tagsVisibleEntry;
  private final GenericEntry bestTargetDistEntry;

  public VisionSubsystem(Drive drive) {
    this.drive = drive;
    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    SmartDashboard.putData("Vision/Field", m_field);

    // Persistent Shuffleboard widgets
    backPresentEntry =
        visionTab
            .add("BACK PRESENT", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(0, 0)
            .withSize(2, 1)
            .getEntry();
    visionHealthyEntry =
        visionTab
            .add("Vision Healthy", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(2, 0)
            .withSize(2, 1)
            .getEntry();
    activeSourceEntry =
        visionTab
            .add("ActiveSource", "None")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 1)
            .withSize(2, 1)
            .getEntry();
    tagsVisibleEntry =
        visionTab
            .add("TagsVisible", 0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(2, 1)
            .withSize(1, 1)
            .getEntry();
    bestTargetDistEntry =
        visionTab
            .add("BestTargetDist", 0.0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(3, 1)
            .withSize(2, 1)
            .getEntry();

    frontCam = new PhotonCamera(FRONT_NAME);
    backCam = new PhotonCamera(BACK_NAME);
    // leftCam = new PhotonCamera(LEFT_NAME);
    // rightCam = new PhotonCamera(RIGHT_NAME);

    // 2026 API Constructor: (Layout, RobotToCameraTransform)
    frontEst =
        new PhotonPoseEstimator(
            fieldLayout, createTrf(cam1_x, cam1_y, cam1_z, cam1_roll, cam1_pitch, cam1_yaw));
    backEst =
        new PhotonPoseEstimator(
            fieldLayout, createTrf(cam2_x, cam2_y, cam2_z, cam2_roll, cam2_pitch, cam2_yaw));
    // leftEst = new PhotonPoseEstimator(fieldLayout, createTrf(0.0, 12.0, 12.0, 0, 0, 90.0));
    // rightEst = new PhotonPoseEstimator(fieldLayout, createTrf(0.0, -12.0, 12.0, 0, 0, -90.0));
  }

  @Override
  public void periodic() {
    updateCam(frontCam, frontEst, fData, FRONT_NAME);
    updateCam(backCam, backEst, bData, BACK_NAME);

    // updateCam(leftCam, leftEst, lData);
    // updateCam(rightCam, rightEst, rData);
    log();
  }

  private void updateCam(
      PhotonCamera cam, PhotonPoseEstimator est, CameraData data, String camera) {
    long start = System.nanoTime();
    var results = cam.getAllUnreadResults();
    for (var res : results) {
      data.result = res;
      if (!res.hasTargets()) {
        data.pose = Optional.empty();
        continue;
      }

      // Distance Filter
      if (res.getTargets().size() == 1) {
        double dist = res.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();
        if (dist > MAX_SINGLE_TAG_DIST) continue;
      }

      // 1. Use the 2026.2.2 Strategy Method
      est.estimateCoprocMultiTagPose(res)
          .ifPresent(
              estPose -> {
                // Convert 3D estimated pose to 2D for the Swerve Drive
                var estPose2d = estPose.estimatedPose.toPose2d();

                // 2. Validate the pose is physically on the field
                if (isPoseValid(estPose2d)) {
                  data.pose = Optional.of(estPose);

                  // 3. Calculate Average Distance to all visible targets
                  double avgDistance =
                      res.getTargets().stream()
                          .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
                          .average()
                          .orElse(4.0);

                  // 4. Determine Dynamic Trust (Standard Deviations)
                  int tagCount = res.getTargets().size();
                  Vector<N3> dynamicStds;

                  if (tagCount > 1) {
                    // MULTI-TAG: High confidence that increases with more tags
                    // 2 tags = 0.025 base, 3 tags = 0.016 base, 4 tags = 0.0125 base
                    double trustFactor = 0.05 / tagCount;
                    double xyStdDev = trustFactor * Math.pow(avgDistance, 2);

                    // Rotation trust also increases with tag count
                    dynamicStds = VecBuilder.fill(xyStdDev, xyStdDev, 0.5 / tagCount);
                  } else {
                    // SINGLE-TAG: Low confidence (penalty multiplier of 2.0)
                    // This ensures we rely more on Swerve/Gyro when only 1 tag is seen
                    double xyStdDev = 2.0 * Math.pow(avgDistance, 2);
                    dynamicStds = VecBuilder.fill(xyStdDev, xyStdDev, 5.0);
                  }

                  // 5. Apply the measurement to the Drive Subsystem
                  // Uses the synchronized timestamp from the PhotonLib result
                  drive.addVisionMeasurement(
                      estPose2d, estPose.timestampSeconds, dynamicStds, camera);

                  // Update the timestamp for dashboard/logging
                  lastMeasurementTimestamp = Timer.getFPGATimestamp();
                }
                lastTagCount = res.getTargets().size();
              });
    }
    data.loopTimeNanos = System.nanoTime() - start;
  }

  // public int getTagCount() {
  //     // If the last measurement is too old (e.g., > 0.5s), return 0
  //     if (Timer.getFPGATimestamp() - lastMeasurementTimestamp > 0.5) {
  //         return 0;
  //     }
  //     return lastTagCount;
  // }

  public int getTagCount() {
    // 1. Get current robot velocity (linear magnitude)
    // We use the drive subsystem speeds we set up earlier
    var speeds = drive.getChassisSpeeds();
    double velocity =
        Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2));

    // 2. Calculate a Dynamic Timeout
    // If stopped (0m/s), timeout is 0.8s.
    // If sprinting (5m/s), timeout drops to 0.1s.
    double dynamicTimeout = MathUtil.clamp(0.8 - (velocity * 0.14), 0.1, 0.8);

    // 3. Check if the data is "stale" based on that timeout
    double timeSinceLastTag = Timer.getFPGATimestamp() - lastMeasurementTimestamp;

    if (timeSinceLastTag > dynamicTimeout) {
      return 0; // Data is too old for our current speed
    }
    return lastTagCount;
  }

  public void forceOdometryToVision() {
    double speed =
        Math.hypot(
            drive.getActualChassisSpeeds().vxMetersPerSecond,
            drive.getActualChassisSpeeds().vyMetersPerSecond);
    if (speed > MAX_RESET_VELOCITY) return;

    CameraData[] all = {fData, bData};
    for (CameraData d : all) {
      if (d.pose.isPresent() && d.result.getTargets().size() > 1) {
        drive.resetPose(d.pose.get().estimatedPose.toPose2d());
        return;
      }
    }
  }

  private boolean isPoseValid(Pose2d estPose) {
    // 1. Field Boundary Check: 2026 REBUILT field is ~16.5m x 8.2m
    if (estPose.getX() < -0.5
        || estPose.getX() > 17.0
        || estPose.getY() < -0.5
        || estPose.getY() > 8.7) {
      return false;
    }

    return true;
  }

  private Transform3d createTrf(double x, double y, double z, double r, double p, double yw) {
    return new Transform3d(
        new Translation3d(x, y, z), // Variables are already in meters!
        new Rotation3d(
            Units.degreesToRadians(r), Units.degreesToRadians(p), Units.degreesToRadians(yw)));
  }

  private void log() {
    // ── Vision page ────────────────────────────────────────────────────────────

    // 1. Update the main robot pose from your Drive subsystem
    m_field.setRobotPose(drive.getPose());

    // 2. Log Front Camera Estimate (if present)
    if (fData.pose.isPresent()) {
      m_field.getObject("Front Cam Pose").setPose(fData.pose.get().estimatedPose.toPose2d());
    } else {
      // Optional: Move it off-field if lost to "hide" it
      m_field.getObject("Front Cam Pose").setPoses();
    }

    // Update persistent BACK PRESENT
    backPresentEntry.setBoolean(bData.pose.isPresent());

    // 3. Log Back Camera Estimate (if present)
    if (bData.pose.isPresent()) {
      m_field.getObject("Back Cam Pose").setPose(bData.pose.get().estimatedPose.toPose2d());
    } else {
      m_field.getObject("Back Cam Pose").setPoses();
    }

    boolean healthy = (Timer.getFPGATimestamp() - lastMeasurementTimestamp) < VISION_TIMEOUT;
    visionHealthyEntry.setBoolean(healthy);

    // Field2d widget (robot + camera poses) - only update value, don't re-put the widget
    SmartDashboard.putData("Field", m_field);

    logBestCameraSummary();
  }

  private void logBestCameraSummary() {
    CameraData[] allCams = {fData, bData};
    String[] names = {FRONT_NAME, BACK_NAME};

    int bestCamIdx = -1;
    int maxTags = 0;

    // Find the camera seeing the most tags
    for (int i = 0; i < allCams.length; i++) {
      int tagCount = allCams[i].result.getTargets().size();
      if (tagCount > maxTags) {
        maxTags = tagCount;
        bestCamIdx = i;
      }
    }

    if (bestCamIdx != -1) {
      activeSourceEntry.setString(names[bestCamIdx]);
      tagsVisibleEntry.setDouble(maxTags);

      // Get distance of the best target from the best camera
      double dist =
          allCams[bestCamIdx]
              .result
              .getBestTarget()
              .getBestCameraToTarget()
              .getTranslation()
              .getNorm();
      bestTargetDistEntry.setDouble(dist);
    } else {
      activeSourceEntry.setString("None");
      tagsVisibleEntry.setDouble(0);
      bestTargetDistEntry.setDouble(0.0);
    }
  }

  // --- FAST GETTERS (Updated for 4-Camera Data Structure) ---

  public boolean frontCameraHasTargets() {
    return fData.result.hasTargets();
  }

  public boolean backCameraHasTargets() {
    return bData.result.hasTargets();
  }

  // public boolean leftCameraHasTargets() {
  //   return lData.result.hasTargets();
  // }

  // public boolean rightCameraHasTargets() {
  //   return rData.result.hasTargets();
  // }

  public int getFrontCameraBestTargetId() {
    return fData.result.hasTargets() ? fData.result.getBestTarget().getFiducialId() : -1;
  }

  public int getBackCameraBestTargetId() {
    return bData.result.hasTargets() ? bData.result.getBestTarget().getFiducialId() : -1;
  }

  public double getFrontCameraBestTargetArea() {
    return fData.result.hasTargets() ? fData.result.getBestTarget().getArea() : 0.0;
  }

  public double getBackCameraBestTargetArea() {
    return bData.result.hasTargets() ? bData.result.getBestTarget().getArea() : 0.0;
  }

  public PhotonPipelineResult getLastFrontResult() {
    return fData.result;
  }

  public PhotonPipelineResult getLastBackResult() {
    return bData.result;
  }

  public Optional<EstimatedRobotPose> getLastFrontEstimatedPose() {
    return fData.pose;
  }

  public Optional<EstimatedRobotPose> getLastBackEstimatedPose() {
    return bData.pose;
  }

  // These can be calculated based on the result count stored in the cache
  public Matrix<N3, N1> getLastFrontStdDevs() {
    return fData.result.getTargets().size() > 1
        ? MULTI_TAG
        : (DriverStation.isAutonomous() ? SINGLE_TAG_AUTO : SINGLE_TAG_TELEOP);
  }

  public AprilTagFieldLayout getFieldLayout() {
    return fieldLayout;
  }

  // Getters for the Transforms (using the estimators)
  public Transform3d getRobotToFrontCamera() {
    return frontEst.getRobotToCameraTransform();
  }

  public Transform3d getRobotToBackCamera() {
    return backEst.getRobotToCameraTransform();
  }

  /** Get the last update time in microseconds for performance monitoring. */
  public double getFrontCameraUpdateTimeMicros() {
    return fData.loopTimeNanos / 1000.0;
  }

  public double getBackCameraUpdateTimeMicros() {
    return bData.loopTimeNanos / 1000.0;
  }
}
