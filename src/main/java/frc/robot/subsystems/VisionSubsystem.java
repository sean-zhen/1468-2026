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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
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
  private static final String FRONT_NAME = "frontCamera";
  private static final String BACK_NAME = "backCamera";
  private static final String LEFT_NAME = "leftCamera";
  private static final String RIGHT_NAME = "rightCamera";

  // Constants
  private static final double MAX_SINGLE_TAG_DIST = 3.0; // Meters
  private static final double MAX_RESET_VELOCITY = 0.5; // M/s
  private static final double VISION_TIMEOUT = 0.5; // Seconds

  // Confidence Matrices
  private static final Matrix<N3, N1> SINGLE_TAG_TELEOP = VecBuilder.fill(0.1, 0.1, 4.0);
  private static final Matrix<N3, N1> SINGLE_TAG_AUTO = VecBuilder.fill(4.0, 4.0, 8.0);
  private static final Matrix<N3, N1> MULTI_TAG = VecBuilder.fill(0.1, 0.1, 0.25);

  private final PhotonCamera frontCam, backCam, leftCam, rightCam;
  private final PhotonPoseEstimator frontEst, backEst, leftEst, rightEst;
  private final AprilTagFieldLayout fieldLayout;
  private final Drive drive;

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

  public VisionSubsystem(Drive drive) {
    this.drive = drive;
    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    frontCam = new PhotonCamera(FRONT_NAME);
    backCam = new PhotonCamera(BACK_NAME);
    leftCam = new PhotonCamera(LEFT_NAME);
    rightCam = new PhotonCamera(RIGHT_NAME);

    // 2026 API Constructor: (Layout, RobotToCameraTransform)
    frontEst = new PhotonPoseEstimator(fieldLayout, createTrf(13.6, -10.5, 12.125, 0, 0.5, 22.8));
    backEst =
        new PhotonPoseEstimator(fieldLayout, createTrf(13.6, 10.5, 12.125, -0.44, -0.5, -22.8));
    leftEst = new PhotonPoseEstimator(fieldLayout, createTrf(0.0, 12.0, 12.0, 0, 0, 90.0));
    rightEst = new PhotonPoseEstimator(fieldLayout, createTrf(0.0, -12.0, 12.0, 0, 0, -90.0));
  }

  @Override
  public void periodic() {
    updateCam(frontCam, frontEst, fData);
    updateCam(backCam, backEst, bData);
    updateCam(leftCam, leftEst, lData);
    updateCam(rightCam, rightEst, rData);
    log();
  }

  private void updateCam(PhotonCamera cam, PhotonPoseEstimator est, CameraData data) {
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
                  drive.addVisionMeasurement(estPose2d, estPose.timestampSeconds, dynamicStds);

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

    CameraData[] all = {fData, bData, lData, rData};
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

    // 2. Velocity Delta Check: Vision should not jump significantly from current odometry
    // If the jump is > 1.0m, it's likely a "ghost" detection
    double poseJump = estPose.getTranslation().getDistance(drive.getPose().getTranslation());
    if (poseJump > 1.0) {
      return false;
    }

    return true;
  }

  private Transform3d createTrf(double x, double y, double z, double r, double p, double yw) {
    return new Transform3d(
        new Translation3d(
            Units.inchesToMeters(x), Units.inchesToMeters(y), Units.inchesToMeters(z)),
        new Rotation3d(
            Units.degreesToRadians(r), Units.degreesToRadians(p), Units.degreesToRadians(yw)));
  }

  private void log() {
    boolean healthy = (Timer.getFPGATimestamp() - lastMeasurementTimestamp) < VISION_TIMEOUT;
    SmartDashboard.putBoolean("Vision/Healthy", healthy);

    logBestCameraSummary();
  }

  private void logBestCameraSummary() {
    CameraData[] allCams = {fData, bData, lData, rData};
    String[] names = {"Front", "Back", "Left", "Right"};

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
      SmartDashboard.putString("Vision/Summary/ActiveSource", names[bestCamIdx]);
      SmartDashboard.putNumber("Vision/Summary/TagsVisible", maxTags);

      // Get distance of the best target from the best camera
      double dist =
          allCams[bestCamIdx]
              .result
              .getBestTarget()
              .getBestCameraToTarget()
              .getTranslation()
              .getNorm();
      SmartDashboard.putNumber("Vision/Summary/BestTargetDist", dist);
    } else {
      SmartDashboard.putString("Vision/Summary/ActiveSource", "None");
      SmartDashboard.putNumber("Vision/Summary/TagsVisible", 0);
    }
  }

  // --- FAST GETTERS (Updated for 4-Camera Data Structure) ---

  public boolean frontCameraHasTargets() {
    return fData.result.hasTargets();
  }

  public boolean backCameraHasTargets() {
    return bData.result.hasTargets();
  }

  public boolean leftCameraHasTargets() {
    return lData.result.hasTargets();
  }

  public boolean rightCameraHasTargets() {
    return rData.result.hasTargets();
  }

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
