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
  // Dashboard toggle to disable the safety if needed
  private final GenericEntry trenchAutoMutedEntry =
      Shuffleboard.getTab("Shooter")
          .add("Trench Auto Muted", false)
          .withWidget(BuiltInWidgets.kToggleSwitch)
          .withPosition(10, 3)
          .getEntry();

  // Added: A dedicated entry to warn the driver when the safety is active
  private final GenericEntry trenchWarningEntry =
      Shuffleboard.getTab("Shooter")
          .add("TRENCH SAFETY ACTIVE", false)
          .withWidget(BuiltInWidgets.kBooleanBox)
          .withPosition(10, 4)
          .getEntry();

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

    // Update the Dashboard Warning
    boolean overrideMuted = trenchAutoMutedEntry.getBoolean(false);
    trenchWarningEntry.setBoolean(inTrench && !overrideMuted);
    var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

    // 2. Calculate turret field position
    Translation2d turretFieldPos =
        robotPose.getTranslation().plus(Shooter.TURRET_TO_ROBOT.rotateBy(robotPose.getRotation()));

    // 3. Determine shooting target
    Translation2d shootingTarget =
        (alliance == DriverStation.Alliance.Red) ? Shooter.RED_HUB_POS : Shooter.BLUE_HUB_POS;

    if (!getZone(robotPose.getX(), alliance).equals("Alliance")) {
      if (alliance == DriverStation.Alliance.Red) {
        shootingTarget =
            (robotPose.getX() < 4.035) ? Shooter.RED_DEPOT_POS : Shooter.RED_OUTPOST_POS;
      } else {
        shootingTarget =
            (robotPose.getX() < 4.035) ? Shooter.BLUE_OUTPOST_POS : Shooter.BLUE_HUB_POS;
      }
    }

    // 4. Velocity compensation
    ChassisSpeeds robotSpeeds = drive.getActualChassisSpeeds();

    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, robotPose.getRotation());

    // 4. Velocity Leading (Virtual Target)
    double flightTime = turretFieldPos.getDistance(shootingTarget) / fuelVelocity;
    Translation2d virtualTarget =
        shootingTarget.minus(
            new Translation2d(
                fieldSpeeds.vxMetersPerSecond * flightTime,
                fieldSpeeds.vyMetersPerSecond * flightTime));

    // 5. Aim calculation
    Translation2d relTrans = virtualTarget.minus(turretFieldPos);

    double odoTargetAngle = Math.atan2(relTrans.getY(), relTrans.getX());

    // ---> ADDED: Save the result to our variable so we don't calculate it again
    cachedAimAngle = new Rotation2d(odoTargetAngle);

    double odoRot =
        MathUtil.inputModulus(
            (odoTargetAngle / (2 * Math.PI)) - robotPose.getRotation().getRotations(), -0.5, 0.5);

    // shooter.setShooterParams(relTrans.getNorm(), getZone(robotPose.getX(), alliance));
    if (flyWheelRPSOverride.getAsDouble() == DONT_OVERRIDE_VAL)
      shooter.setFlywheelViaTable(relTrans.getNorm(), getZone(robotPose.getX(), alliance));
    else shooter.setFlywheelRPS(flyWheelRPSOverride.getAsDouble());

    // Hood Logic with Trench Override
    if (inTrench && !overrideMuted) {
      // FORCE HOOD DOWN IN TRENCH
      shooter.setHoodPosition(0.0);
    } else {
      if (hoodAngleOverride.getAsDouble() == DONT_OVERRIDE_VAL)
        shooter.setHoodViaTable(relTrans.getNorm(), getZone(robotPose.getX(), alliance));
      else shooter.setHoodPosition(hoodAngleOverride.getAsDouble() / 20.0 * 1.25);
    }

    // double turretFF = -robotSpeeds.omegaRadiansPerSecond / (2 * Math.PI);
    // shooter.setTurretWithFF(odoRot, turretFF); // Using Magic Motion Now
    if (TurretAngleOverride.getAsDouble() == DONT_OVERRIDE_VAL) shooter.setTurretPosition(odoRot);
    // Override value is in degrees, convert to rotations (+/-180 Deg -> +/- 0.5 Rot), so just /
    // 360.0
    else shooter.setTurretPosition(TurretAngleOverride.getAsDouble() / 360.0);

    // ---------------------------------------------------------
    // Elastic Field Visualization
    // ---------------------------------------------------------

    Pose2d turretAimingPose = new Pose2d(turretFieldPos, new Rotation2d(odoTargetAngle));

    // Turret
    field.getObject("Turret").setPose(turretAimingPose);

    // Virtual target (lead compensated)
    field.getObject("VirtualTarget").setPose(new Pose2d(virtualTarget, new Rotation2d()));

    // Real target
    field.getObject("RealTarget").setPose(new Pose2d(shootingTarget, new Rotation2d()));

    // Shot trajectory line
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
