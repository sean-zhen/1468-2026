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
    {3.96, 6.8, 5.25, 7.9}, // First trench
    {3.96, 0, 5.25, 1.2}, // Second trench
    {11.3, 0, 12.5, 1.2}, // Third trench
    {11.3, 6.8, 12.5, 7.9} // Fourth trench
  };

  // Overrides
  private final DoubleSupplier flyWheelRPSOverride;
  private final DoubleSupplier hoodAngleOverride;
  private final DoubleSupplier TurretAngleOverride;

  // ---> ADDED: Variable to store the calculated angle
  private Rotation2d cachedAimAngle = new Rotation2d();

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

  @Override
  public void execute() {

    // 1. Sync CAN signals
    BaseStatusSignal.waitForAll(0.020, allSignals);

    Pose2d robotPose = drive.getPose();
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
    Translation2d turretOffsetField = Shooter.TURRET_TO_ROBOT.rotateBy(rot);
    Translation2d turretFieldPos = robotPose.getTranslation().minus(turretOffsetField);

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

    // Rotational velocity contribution: ω × r
    Translation2d turretRotVel =
        new Translation2d(
            -robotFieldSpeeds.omegaRadiansPerSecond * turretOffsetField.getY(),
            robotFieldSpeeds.omegaRadiansPerSecond * turretOffsetField.getX());

    // Total turret velocity in field frame
    Translation2d turretVel =
        new Translation2d(robotFieldSpeeds.vxMetersPerSecond, robotFieldSpeeds.vyMetersPerSecond)
            .plus(turretRotVel);

    // ------------------------------
    // 5. Compute virtual target (lead compensation)
    // ------------------------------
    double distance = turretFieldPos.getDistance(shootingTarget);
    double flightTime = distance / fuelVelocity;

    // Correct lead sign: subtract robot/turret velocity
    Translation2d virtualTarget = shootingTarget.minus(turretVel.times(flightTime));

    // ------------------------------
    // 6. Compute aim angle
    // ------------------------------
    Translation2d relTrans = virtualTarget.minus(turretFieldPos);
    double targetAngle = Math.atan2(relTrans.getY(), relTrans.getX());

    cachedAimAngle = new Rotation2d(targetAngle);

    double turretRot =
        MathUtil.inputModulus(
            (targetAngle / (2 * Math.PI)) - robotPose.getRotation().getRotations(), -0.5, 0.5);

    // Apply turret angle (unless overridden)
    if (TurretAngleOverride.getAsDouble() == DONT_OVERRIDE_VAL)
      shooter.setTurretPosition(turretRot);
    else shooter.setTurretPosition(TurretAngleOverride.getAsDouble() / 360.0);

    // ------------------------------
    // 7. Flywheel + Hood (lead‑compensated)
    // ------------------------------
    double shotDistance = virtualTarget.getDistance(turretFieldPos);
    String zone = getZone(robotPose.getX(), alliance);

    // Flywheel
    if (flyWheelRPSOverride.getAsDouble() == DONT_OVERRIDE_VAL)
      shooter.setFlywheelViaTable(shotDistance, zone);
    else shooter.setFlywheelRPS(flyWheelRPSOverride.getAsDouble());

    // Hood
    if (inTrench) {
      shooter.setHoodPosition(0.0);
    } else {
      if (hoodAngleOverride.getAsDouble() == DONT_OVERRIDE_VAL)
        shooter.setHoodViaTable(shotDistance, zone);
      else shooter.setHoodPosition(hoodAngleOverride.getAsDouble() / 20.0 * 1.25);
    }

    // ------------------------------
    // 8. Field Visualization
    // ------------------------------
    Pose2d turretAimingPose = new Pose2d(turretFieldPos, new Rotation2d(targetAngle));

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
