// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  //////////////////////////    VISION / AUTO ALIGNMENT
  // /////////////////////////////////////////

  public static final class AutoAlign {
    // Rotation PID (for aligning robot to face AprilTags)
    public static final double ROTATION_kP = 5.0;
    public static final double ROTATION_kI = 0.0;
    public static final double ROTATION_kD = 0.2;
    public static final double ROTATION_TOLERANCE_DEG = 2.0;

    // Distance PID (for maintaining distance from AprilTags)
    public static final double DISTANCE_kP = 2.0;
    public static final double DISTANCE_kI = 0.0;
    public static final double DISTANCE_kD = 0.1;
    public static final double DISTANCE_TOLERANCE_METERS = 0.3;

    // Target distances
    public static final double HUB_DISTANCE_METERS = Units.feetToMeters(10.0);
    public static final double VISION_DISTANCE_THRESHOLD_METERS =
        Units.feetToMeters(15.0); // TODO: TA - 15ft seems far
  }

  //////////////////////////    SHOOTER     /////////////////////////////////////////

  public static final class Shooter {
    public static final int FLYWHEEL_LEAD_ID = 51;
    public static final int FLYWHEEL_FOLLOWER_ID = 52;
    public static final int HOOD_MOTOR_ID = 54;
    public static final int TURRET_MOTOR_ID = 53;

    // TODO: Gear ratios need to be determined
    public static final double HOOD_GEAR_RATIO = 1.0;
    public static final double TURRET_GEAR_RATIO = 1.0;

    public static final double FIELD_LENGTH = 16.54; // meters
    // 2026 Hub Centers (Verify exact X from official CAD/Drawings)

    public static final Translation2d BLUE_HUB_POS = new Translation2d(4.625, 4.035);
    public static final Translation2d RED_HUB_POS = new Translation2d(11.915, 4.035);

    public static final Translation2d BLUE_DEPOT_POS = new Translation2d(1.0, 6.0);
    public static final Translation2d RED_DEPOT_POS = new Translation2d(15.0, 2.0);
    public static final Translation2d BLUE_OUTPOST_POS = new Translation2d(1.0, 1.0);
    public static final Translation2d RED_OUTPOST_POS = new Translation2d(15.0, 7.0);
    // Turret Offset from Robot Center (Inches to Meters)
    public static final double TURRET_OFFSET_X = 0.0; // 5.5 * 0.0254; // TODO: TA get real offset
    public static final double TURRET_OFFSET_Y = 0.0; // 5.5 * 0.0254;
    public static final Translation2d TURRET_TO_ROBOT =
        new Translation2d(TURRET_OFFSET_X, TURRET_OFFSET_Y);

    // Vision
    public static final String CAMERA_NAME = "ShooterCam";

    //////////////////////////    FLYWHEEL     /////////////////////////////////////////
    // TODO: Tune these values (flywheel)

    public static final double SHOOT_kP = 0.89;
    public static final double SHOOT_kI = 0.0;
    public static final double SHOOT_kD = 0.0;
    public static final double SHOOT_kV = 0.37;
    public static final double SHOOT_kA = 0.43;

    // public static final double FLYWHEEL_TARGET_RPS = 50.0; // NEED TO INSERT
    public static final double VELOCITY_TOLERANCE_RPS = 2.0;

    //////////////////////////    HOOD     /////////////////////////////////////////
    // Hood PID and soft stops (hood)

    public static final double HOOD_kP = 0.5;
    public static final double HOOD_kI = 0.0;
    public static final double HOOD_kD = 0.0;
    public static final double HOOD_kV = 0.0;
    public static final double HOOD_TOP_SOFT_LIMIT_ROT = 10.0;
    public static final double HOOD_BOTTOM_SOFT_LIMIT_ROT = 0.0;
    public static final double HOOD_TRACKING_TOLERANCE_DEG = 1.0; // DEGREE TOLERANCE
    public static final double HOOD_DEG_PER_ROTATION = 360.0; // DEGREE TOLERANCE

    //////////////////////////    TURRET     /////////////////////////////////////////
    // Turret PID and soft stops (Turret)

    public static final double TURRET_kP = 2.5;
    public static final double TURRET_kI = 0.0;
    public static final double TURRET_kD = 0.0;
    public static final double TURRET_kV = 0.0;

    public static final double TURRET_LIMIT_ROT = 135.0 / 360.0; // +/- .375 Rotstions
    // This multiplier is used to smooth the turret motion over angles that cant be used due to
    // wiring
    public static final double TURRET_CLAMP_FACTOR = 0.5 / (0.5 - TURRET_LIMIT_ROT);
    public static final double TURRET_RIGHT_SOFT_LIMIT_ROT = TURRET_LIMIT_ROT;
    public static final double TURRET_LEFT_SOFT_LIMIT_ROT = -TURRET_LIMIT_ROT;

    public static final double TURRET_TRACKING_TOLERANCE_DEG = 1.0; // DEGREE TOLERANCE
    public static final double TURRET_MAX_VELOCITY_RPS = 5.0; // Max turret rotation speed
  }

  //////////////////////////    KICKER     /////////////////////////////////////////

  public static final class Kicker {
    public static final int KICKER_MOTOR_ID = 42;
    public static final double KICKER_TARGET_RPS = 30.0;
    public static final double KICKER_GEAR_RATIO = 3.0; // NEED TO INSERT

    // TODO: Tune these values
    public static final double kP = 0.89;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.37;
    public static final double kA = 0.43;

    public static final double VELOCITY_TOLERANCE_RPS = 5.0;
  }

  //////////////////////////    HARVESTER     /////////////////////////////////////////

  public static final class Harvester {
    public static final int DEPLOY_MOTOR_ID = 31;
    // THE DEPLOY MECHANISM HAS A 20:1 GEAR RATIO AND A 20:1 REDUCTIONS ON THE BELT DRIVE
    // THE GEAR RATIO IS HANDED OFF TO THE MOTOR CONFIGURATION
    public static final double DEPLOY_GEAR_RATIO = 20.0;
    // THE EXTERNAL BELT RATION IS INCORPORATED WHEN CONVERTING TO ROTATIONS
    public static final double DEPLOY_DEGREES_TO_ROTATIONS = 2.5 / 360.0;

    public static final double DEPLOY_START_ANGLE = 0.0; // Degrees
    public static final double DEPLOY_IN_ANGLE = 10.0; // Degrees
    public static final double DEPLOY_OUT_ANGLE = 100.0; // Degrees

    public static final int SPIN_MOTOR_ID = 32;
    public static final double SPIN_GEAR_RATIO = 3.0; // NEED TO INSERT
    public static final double SPIN_TARGET_RPS = -30.0;

    // TODO: Tune these values
    public static final double deploykP = 0.89;
    public static final double deploykI = 0.0;
    public static final double deploykD = 0.0;
    public static final double deploykV = 0.37;

    public static final double spinkP = 0.89;
    public static final double spinkI = 0.0;
    public static final double spinkD = 0.0;
    public static final double spinkV = 0.37;
  }

  //////////////////////////    INDEXER     /////////////////////////////////////////

  public static final class Indexer {
    public static final int MOTOR_ID = 41;
    public static final double GEAR_RATIO = 1.0; // NEED TO INSERT
    public static final double TARGET_RPS = 90.0; // NEED TO INSERT
    public static final double[] POSITIONS = {
      0.0, 10.0, 20.0, 30.0, 40.0, 50.0
    }; // NEED TO INSERT (positions in rotations)

    // TODO: Tune these values
    public static final double kP = 0.189; // was.89
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.37;
  }

  //////////////////////////    CLIMBER     /////////////////////////////////////////'

  public static final class Climber {
    public static final int LEFT_MOTOR_ID = 61;
    //   public static final int RIGHT_MOTOR_ID = 62;  //TA TODO: 1 or 2 Clmbr Motors???

    public static final double FORWARD_SOFT_LIMIT_ROT = 200.0; // rotations
    public static final double REVERSE_SOFT_LIMIT_ROT = 0.0; // rotations
    public static final double UP_SPEED = 0.5;
    public static final double DOWN_SPEED = -0.5;
    public static final double SMALL_MOVE_ROT = 0.5;
    public static final double HOME_POSITION_ROT = 0.0;

    // TODO: Tune these values
    public static final double kP = 0.60; // 60 way too high
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.37;
  }

  /////////////////////////// LED Constants
  public static class Climb {
    public static final double TOWER_LEFT_X = 8.27;
    public static final double TOWER_LEFT_Y = 1.2;
    public static final double TOWER_RIGHT_X = 8.27;
    public static final double TOWER_RIGHT_Y = 7.0;
  }
}
