package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Shooter;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drive.Drive;

public class PrepareShooterCmd extends Command {
  private final ShooterSubsystem shooter;
  private final Drive drive;

  public PrepareShooterCmd(ShooterSubsystem shooter, Drive drive) {
    this.shooter = shooter;
    this.drive = drive;
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    Pose2d robotPose = drive.getPose();

    // 1. Transform Robot Pose to Turret Pose
    Translation2d turretFieldPos =
        robotPose.getTranslation().plus(Shooter.TURRET_TO_ROBOT.rotateBy(robotPose.getRotation()));

    var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

    // Determine Target to shoot at based on alliance color and position on field
    Translation2d shootingTarget =
        (alliance == DriverStation.Alliance.Red) ? Shooter.RED_HUB_POS : Shooter.BLUE_HUB_POS;

    if (getZone(robotPose.getX(), alliance) != "Alliance") {
      if (alliance == DriverStation.Alliance.Red) {
        shootingTarget =
            (robotPose.getX() < 4.035) ? Shooter.RED_DEPOT_POS : Shooter.RED_OUTPOST_POS;
      } else {
        shootingTarget =
            (robotPose.getX() < 4.035) ? Shooter.BLUE_OUTPOST_POS : Shooter.BLUE_HUB_POS;
      }
    }

    // TODO TA: Not sure if this is correct???
    // FIX: Manually convert robot-relative speeds to field-relative
    // Replace 'getRobotRelativeSpeeds()' with your subsystem's actual getter name if different
    // ChassisSpeeds robotSpeeds = drive.getRobotRelativeSpeeds();
    // ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds,
    // robotPose.getRotation());

    ChassisSpeeds robotSpeeds = drive.getChassisSpeeds();
    ChassisSpeeds fieldSpeeds = robotSpeeds; // TODO TA: Not sure if this is correct???

    // 2. Velocity Leading (Virtual Target)
    double flightTime =
        turretFieldPos.getDistance(shootingTarget) / 20.0; // TODO: TA - Optimize Shooting sp m/sec
    Translation2d virtualTarget =
        shootingTarget.minus(
            new Translation2d(
                fieldSpeeds.vxMetersPerSecond * flightTime,
                fieldSpeeds.vyMetersPerSecond * flightTime));

    // 3. Aiming Logic
    Translation2d relTrans = virtualTarget.minus(turretFieldPos);
    double odoTargetAngle = Math.atan2(relTrans.getY(), relTrans.getX());
    double odoRot =
        MathUtil.inputModulus(
            (odoTargetAngle / (2 * Math.PI)) - robotPose.getRotation().getRotations(), -0.5, 0.5);

    // 4. Final Updates
    double finalRot = shooter.getVisionCorrectedRotation(odoRot);
    double turretFF = -robotSpeeds.omegaRadiansPerSecond / (2 * Math.PI);

    shooter.setShooterParams(relTrans.getNorm(), getZone(robotPose.getX(), alliance));
    shooter.setTurretWithFF(finalRot, turretFF);
  }

  private String getZone(double x, DriverStation.Alliance alliance) {
    if (alliance == DriverStation.Alliance.Blue) {
      if (x < 5.5) return "Alliance";
      if (x > 11.0) return "Opposition";
    } else { // Red Alliance
      if (x > 11.0) return "Alliance";
      if (x < 5.5) return "Opposition";
    }
    return "Neutral";
  }
}
