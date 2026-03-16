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

    // Sentinel Value: If a parameter equals this, use Auto-Calculation
    public static final double DONT_OVERRIDE_VAL = 999.0;

    public static final double FIELD_LENGTH = 16.54; // meters
    // 2026 Hub Centers (Verify exact X from official CAD/Drawings)

    public static final Translation2d BLUE_HUB_POS = new Translation2d(4.625, 4.035);
    public static final Translation2d RED_HUB_POS = new Translation2d(11.915, 4.035);

    public static final Translation2d BLUE_DEPOT_POS = new Translation2d(1.0, 6.0);
    public static final Translation2d RED_DEPOT_POS = new Translation2d(15.0, 2.0);
    public static final Translation2d BLUE_OUTPOST_POS = new Translation2d(1.0, 1.0);
    public static final Translation2d RED_OUTPOST_POS = new Translation2d(15.0, 7.0);

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

    // *** The Hood Gear has 22 teeth and the Hood itself has 29, so using approx 27 teeth
    public static final double HOOD_TOP_SOFT_LIMIT_ROT = 1.25;
    public static final double HOOD_BOTTOM_SOFT_LIMIT_ROT = 0.0;
    public static final double HOOD_TRACKING_TOLERANCE_ROT =
        HOOD_TOP_SOFT_LIMIT_ROT * 0.02; // 2% error tolerance
    public static final double HOOD_DEG_PER_ROTATION =
        20.0 / HOOD_TOP_SOFT_LIMIT_ROT; // DEGREE TOLERANCE
  }

  //////////////////////////    KICKER     /////////////////////////////////////////

  public static final class Kicker {
    public static final int KICKER_MOTOR_ID = 42;
    public static final double KICKER_TARGET_RPS = 45.0;
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
    public static final double DEPLOY_IN_ANGLE = 2.0; // Degrees  was 12
    public static final double DEPLOY_OUT_ANGLE = 100.0; // Degrees

    public static final int SPIN_MOTOR_ID = 32;
    public static final double SPIN_GEAR_RATIO = 3.0; // NEED TO INSERT
    public static final double SPIN_TARGET_RPS =
        -20.0; // even .75 not fast enoughwas 25 TODO: TA - using old hopper have to retune
    // direction)

    // TODO: Tune these values
    public static final double deploykP = 0.89;
    public static final double deploykI = 0.0;
    public static final double deploykD = 0.0;
    public static final double deploykV = 0.37;

    public static final double spinkP = 0.89;
    public static final double spinkI = 0.0;
    public static final double spinkD = 0.0;
    public static final double spinkV = 0.37;

    // Default homing/current-detection threshold for deploy (Amps). Use this for
    // commands that detect mechanical stops via stator current.
    public static final double DEPLOY_HOMING_CURRENT_AMPS = 35.0;
    // Motion Magic tuning for deploy (mechanism output RPS)
    // Cruise velocity in rotations-per-second (mechanism output)
    public static final double DEPLOY_MM_CRUISE_RPS = 4.0;
    // Motion Magic acceleration (rotations-per-second-per-second)
    public static final double DEPLOY_MM_ACCEL_RPSPS = 2.0;
    // Motion Magic jerk (rotations-per-second-per-second-per-second) - currently unused
    public static final double DEPLOY_MM_JERK = 0.0;
    // How many consecutive cycles the current must exceed the threshold to declare a hit
    public static final int DEPLOY_HOMING_CONSECUTIVE_CYCLES = 4;
    // Timeout for homing operation (seconds). <= 0 disables timeout.
    public static final double DEPLOY_HOMING_TIMEOUT_S = 2.0;
    // If true, zero the deploy encoder when homing finishes due to current-detection
    public static final boolean DEPLOY_HOMING_ZERO_ON_STOP = true;
  }

  //////////////////////////    INDEXER     /////////////////////////////////////////

  public static final class Indexer {
    public static final int MOTOR_ID = 41;
    public static final double GEAR_RATIO = 1.0; // NEED TO INSERT
    public static final double TARGET_RPS =
        22.5 * 1.5; // 0.75; // NEED TO INSERT //22.5// was 45, too fast???
    public static final double[] POSITIONS = {
      0.0, 10.0, 20.0, 30.0, 40.0, 50.0
    }; // NEED TO INSERT (positions in rotations)

    // TODO: Tune these values
    public static final double kP = 0.189; // was.89
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
