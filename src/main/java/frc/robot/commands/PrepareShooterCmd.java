package frc.robot.commands;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Shooter;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class PrepareShooterCmd extends Command {
  private final ShooterSubsystem shooter;
  private final Drive drive;

  private final List<BaseStatusSignal> allSignals = new ArrayList<>();

  public PrepareShooterCmd(ShooterSubsystem shooter, Drive drive) {
    this.shooter = shooter;
    this.drive = drive;
    addRequirements(shooter);

    // Sync List: 8 Swerve motors + 1 Gyro + 3 Shooter motors = 12 signals
    allSignals.addAll(drive.getSignals());
    allSignals.addAll(shooter.getSignals());
  }

  @Override
  public void execute() {

    // 1. SYNC HEARTBEAT: Wait for fresh CAN packets
    BaseStatusSignal.waitForAll(0.020, allSignals);

    Pose2d robotPose = drive.getPose();

    var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

    // 2. Transform Robot Pose to Turret Pose
    Translation2d turretFieldPos =
        robotPose.getTranslation().plus(Shooter.TURRET_TO_ROBOT.rotateBy(robotPose.getRotation()));

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

    // 3. Velocity Compensation (Field-Relative)
    // Get speeds relative to the robot's front
    ChassisSpeeds robotSpeeds = drive.getChassisSpeeds();

    // Convert robot-relative speeds to field-relative speeds using the robot's rotation
    // This ensures that 'forward' on the robot is translated to the correct field direction
    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, robotPose.getRotation());

    // 4. Velocity Leading (Virtual Target)
    double flightTime =
        turretFieldPos.getDistance(shootingTarget) / 20.0; // TODO: TA - Optimize Shooting sp m/sec
    Translation2d virtualTarget =
        shootingTarget.minus(
            new Translation2d(
                fieldSpeeds.vxMetersPerSecond * flightTime,
                fieldSpeeds.vyMetersPerSecond * flightTime));

    // 5. Horizontal Aim & Anti-Spin Feedforward Logic
    Translation2d relTrans = virtualTarget.minus(turretFieldPos);
    double odoTargetAngle = Math.atan2(relTrans.getY(), relTrans.getX());
    double odoRot =
        MathUtil.inputModulus(
            (odoTargetAngle / (2 * Math.PI)) - robotPose.getRotation().getRotations(), -0.5, 0.5);

    double turretFF = -robotSpeeds.omegaRadiansPerSecond / (2 * Math.PI);

    shooter.setShooterParams(relTrans.getNorm(), getZone(robotPose.getX(), alliance));
    shooter.setTurretWithFF(odoRot, turretFF);

    Logger.recordOutput("Shooter/VirtualTarget", virtualTarget);
    Logger.recordOutput("Shooter/RealHub", shootingTarget); // For reference
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
