// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "Camera_1";
  public static String camera1Name = "Camera_2";

  public static double cam1_x = Units.inchesToMeters(13);
  public static double cam1_y = Units.inchesToMeters(-12);
  public static double cam1_z = Units.inchesToMeters(43.5);
  public static double cam1_roll = Units.inchesToMeters(0.0);
  public static double cam1_pitch = Units.inchesToMeters(0.0);
  public static double cam1_yaw = Units.inchesToMeters(0.0);

  public static double cam2_x = Units.inchesToMeters(13);
  public static double cam2_y = Units.inchesToMeters(-6);
  public static double cam2_z = Units.inchesToMeters(43.5);
  public static double cam2_roll = Units.inchesToMeters(0.0);
  public static double cam2_pitch = Units.inchesToMeters(0.0);
  public static double cam2_yaw = Units.inchesToMeters(0.0);

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCamera0 =
      new Transform3d(cam1_x, cam1_y, cam1_z, new Rotation3d(cam1_roll, cam1_pitch, cam1_yaw));
  public static Transform3d robotToCamera1 =
      new Transform3d(cam2_x, cam2_y, cam2_z, new Rotation3d(cam2_roll, cam2_pitch, cam2_yaw));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
