package frc.robot.commands;

import static frc.robot.Constants.Shooter.*;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Shooter;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

public class PrepareShooterCmd extends Command {

  private final ShooterSubsystem shooter;
  private final Drive drive;

  private final double fuelVelocity = 20.0; // TODO: TA - Empirically Optimize Shooting sp m/sec

  private final List<BaseStatusSignal> allSignals = new ArrayList<>();
  private final Field2d field = new Field2d();

  // Trench coordinates: (x1, y1) to (x2, y2)
  private final double[][] trenches = {
    {3.0, 6.5, 6.0, 8.0}, // Blue Left Trench
    {3.0, 0, 6.0, 1.5}, // Blue Right Trench
    {10.5, 0.0, 13.5, 1.5}, // Red Right trench
    {10.5, 6.5, 13.5, 8.0} // Red Left Trench
  };

  // Overrides
  private final DoubleSupplier flyWheelRPSOverride;
  private final DoubleSupplier hoodAngleOverride;
  private final DoubleSupplier TurretAngleOverride;

  private boolean shootingMode = false;

  // ---> ADDED: Variable to store the calculated angle
  private Rotation2d cachedAimAngle = new Rotation2d();

  private Translation2d cachedVirtualTarget = new Translation2d();
  private Translation2d filteredTarget = new Translation2d();

  // Shuffleboard entry to show whether aiming is active
  private static final GenericEntry aimingActiveEntry =
      Shuffleboard.getTab("Shooter")
          .add("Aiming Active", false)
          .withWidget(BuiltInWidgets.kBooleanBox)
          .withPosition(9, 2)
          .withSize(1, 1)
          .getEntry();

  public PrepareShooterCmd(
      ShooterSubsystem shooter,
      Drive drive,
      DoubleSupplier flyWheelRPSOverride,
      DoubleSupplier hoodAngleOverride,
      DoubleSupplier TurretAngleOverride) {
    this.shooter = shooter;
    this.drive = drive;
    this.flyWheelRPSOverride = flyWheelRPSOverride;
    this.hoodAngleOverride = hoodAngleOverride;
    this.TurretAngleOverride = TurretAngleOverride;

    addRequirements(shooter);

    var shtrTab = Shuffleboard.getTab("Shooter");
    // Publish Field2d to the Shooter Elastic tab
    //    shtrTab.add("Field", field).withWidget(BuiltInWidgets.kField).withPosition(0,
    // 6).withSize(6, 4);

    allSignals.addAll(drive.getSignals());
    allSignals.addAll(shooter.getSignals());
  }

  // ---> ADDED: Getter so the Drive command can read the result
  public Rotation2d getAimAngle() {
    return cachedAimAngle;
  }

  public Translation2d getVirtualTarget() {
    // return cachedVirtualTarget;
    return filteredTarget;
  }

  @Override
  public void execute() {

    // 1. Sync CAN signals
    BaseStatusSignal.waitForAll(0.020, allSignals);

    Pose2d robotPose = drive.getPose();

    ChassisSpeeds speeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            drive.getActualChassisSpeeds(), robotPose.getRotation());

    double predictionTime = 0.04;

    Translation2d predicted =
        robotPose
            .getTranslation()
            .plus(
                new Translation2d(
                    speeds.vxMetersPerSecond * predictionTime,
                    speeds.vyMetersPerSecond * predictionTime));

    robotPose = new Pose2d(predicted, robotPose.getRotation());
    Translation2d pos = robotPose.getTranslation();
    field.setRobotPose(robotPose);

    // 2. Check for Trench collision
    boolean inTrench = false;
    for (double[] trench : trenches) {
      if (pos.getX() >= trench[0]
          && pos.getX() <= trench[2]
          && pos.getY() >= trench[1]
          && pos.getY() <= trench[3]) {
        inTrench = true;
        break;
      }
    }

    var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

    // ------------------------------
    // 2. Turret field position
    // ------------------------------
    Rotation2d rot = robotPose.getRotation();
    Translation2d robotFieldPos = robotPose.getTranslation();

    // ------------------------------
    // 3. Determine shooting target
    // ------------------------------
    Translation2d shootingTarget =
        (alliance == DriverStation.Alliance.Red) ? Shooter.RED_HUB_POS : Shooter.BLUE_HUB_POS;

    if (!getZone(robotPose.getX(), alliance).equals("Alliance")) {
      if (alliance == DriverStation.Alliance.Red) {
        shootingTarget =
            (robotPose.getY() < 4.035) ? Shooter.RED_DEPOT_POS : Shooter.RED_OUTPOST_POS;
      } else {
        shootingTarget =
            (robotPose.getY() < 4.035) ? Shooter.BLUE_OUTPOST_POS : Shooter.BLUE_DEPOT_POS;
      }
    }

    // ------------------------------
    // 4. Compute turret velocity in field frame
    // ------------------------------
    ChassisSpeeds robotFieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(drive.getActualChassisSpeeds(), rot);

    // Total robot velocity in field frame
    Translation2d turretVel =
        new Translation2d(robotFieldSpeeds.vxMetersPerSecond, robotFieldSpeeds.vyMetersPerSecond);

    // ------------------------------
    // 5. Compute virtual target (lead compensation)
    // ------------------------------
    double distance = robotFieldPos.getDistance(shootingTarget);
    double flightTime = distance / fuelVelocity;

    // Correct lead sign: subtract robot/turret velocity
    // Translation2d virtualTarget = shootingTarget.minus(turretVel.times(flightTime));
    cachedVirtualTarget = shootingTarget.minus(turretVel.times(flightTime));
    // Translation2d virtualTarget = cachedVirtualTarget;
    // filteredTarget = filteredTarget.interpolate(cachedVirtualTarget, 0.1);
    filteredTarget = filteredTarget.interpolate(cachedVirtualTarget, 0.5);

    Translation2d virtualTarget = filteredTarget;

    // ------------------------------
    // 6. Compute aim angle
    // ------------------------------
    Translation2d relTrans = virtualTarget.minus(robotFieldPos);
    double targetAngle = Math.atan2(relTrans.getY(), relTrans.getX());

    cachedAimAngle = new Rotation2d(targetAngle);

    double robotRot =
        MathUtil.inputModulus(
            (targetAngle / (2 * Math.PI)) - robotPose.getRotation().getRotations(), -0.5, 0.5);

    // // Apply turret angle (unless overridden)
    // if (TurretAngleOverride.getAsDouble() == DONT_OVERRIDE_VAL)
    //   shooter.setTurretPosition(turretRot);
    // else shooter.setTurretPosition(TurretAngleOverride.getAsDouble() / 360.0);

    // ------------------------------
    // 7. Flywheel + Hood (lead‑compensated)
    // ------------------------------
    double shotDistance = virtualTarget.getDistance(robotFieldPos);
    String zone = getZone(robotPose.getX(), alliance);

    // Flywheel
    if (flyWheelRPSOverride.getAsDouble() == DONT_OVERRIDE_VAL)
      shooter.setFlywheelViaTable(shotDistance, zone);
    else shooter.setFlywheelRPS(flyWheelRPSOverride.getAsDouble());

    // Hood
    //   if (inTrench) {

    // if (!shootingMode) {
    if (false) {
      // ALWAYS Zero / safe unless actively shooting
      shooter.setHoodPosition(0.0);
    } else {
      if (hoodAngleOverride.getAsDouble() == DONT_OVERRIDE_VAL)
        shooter.setHoodViaTable(shotDistance, zone);
      else shooter.setHoodPosition(hoodAngleOverride.getAsDouble()); // / 20.0 * 1.25
    }

    // ------------------------------
    // 8. Field Visualization
    // ------------------------------
    Pose2d turretAimingPose = new Pose2d(robotFieldPos, new Rotation2d(targetAngle));

    field.getObject("Turret").setPose(turretAimingPose);
    field.getObject("VirtualTarget").setPose(new Pose2d(virtualTarget, new Rotation2d()));
    field.getObject("RealTarget").setPose(new Pose2d(shootingTarget, new Rotation2d()));
    field
        .getObject("ShotTrajectory")
        .setPoses(turretAimingPose, new Pose2d(virtualTarget, new Rotation2d()));
  }

  @Override
  public void initialize() {
    // Mark aiming as active on Shuffleboard
    aimingActiveEntry.setBoolean(true);

    // Initialize filter to current target to avoid jump
    filteredTarget = cachedVirtualTarget;
  }

  private String getZone(double x, DriverStation.Alliance alliance) {
    if (alliance == DriverStation.Alliance.Blue) {
      if (x < 4.5) return "Alliance"; //
      if (x > 12.0) return "Opposition";
    } else {
      if (x > 12.0) return "Alliance";
      if (x < 4.5) return "Opposition";
    }
    return "Neutral";
  }

  public void setShootingMode(boolean shooting) {
    shootingMode = shooting;
  }

  // inside PrepareShooterCmd
  private boolean lastAligned = false;

  public boolean isAligned() {
    Rotation2d robotRot = drive.getRotation();

    // Compute angular error, wrapped to [-pi, pi]
    double error = MathUtil.angleModulus(cachedAimAngle.minus(robotRot).getRadians());

    // Current robot angular speed
    double angularVel = drive.getActualChassisSpeeds().omegaRadiansPerSecond;

    // Alignment thresholds
    double ANGLE_TOLERANCE_RAD = Math.toRadians(2.0);
    double ANGULAR_VEL_TOLERANCE_RAD = Math.toRadians(10.0);

    // Base aligned check
    boolean currentlyAligned =
        Math.abs(error) < ANGLE_TOLERANCE_RAD && Math.abs(angularVel) < ANGULAR_VEL_TOLERANCE_RAD;

    // Hysteresis: once aligned, require slightly bigger error to mark unaligned
    if (lastAligned) {
      currentlyAligned =
          Math.abs(error) < Math.toRadians(3.0) && Math.abs(angularVel) < Math.toRadians(12.0);
    }

    // Store for next cycle
    lastAligned = currentlyAligned;
    return currentlyAligned;
  }

  @Override
  public void end(boolean interrupted) {
    // Mark aiming inactive and ensure shooter stops when the command ends
    aimingActiveEntry.setBoolean(false);
    shooter.stop();
  }

  @Override
  public boolean isFinished() {
    // Keep running until explicitly interrupted (button release will cancel)
    return false;
  }
}
