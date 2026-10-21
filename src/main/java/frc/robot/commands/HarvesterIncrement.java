package frc.robot.commands;

import static frc.robot.Constants.Harvester.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HarvesterSubsystem;

public class HarvesterIncrement extends Command {
  private final HarvesterSubsystem harvester;
  private final double incrementDegrees;
  private final double toleranceDegrees;

  private double targetDegrees;
  private boolean spinStarted = false;

  public HarvesterIncrement(
      HarvesterSubsystem harvester, double incrementDegrees, double toleranceDegrees) {

    this.harvester = harvester;
    this.incrementDegrees = incrementDegrees;
    this.toleranceDegrees = toleranceDegrees;

    addRequirements(harvester);
  }

  @Override
  public void initialize() {
    double currentDegrees = harvester.getDeployPositionDegrees();

    targetDegrees = currentDegrees + incrementDegrees;

    harvester.setHarvDeployMagicMoPos(targetDegrees * DEPLOY_DEGREES_TO_ROTATIONS);

    spinStarted = false;
  }

  @Override
  public void execute() {
    double currentDegrees = harvester.getDeployPositionDegrees();

    if (!spinStarted
        && targetDegrees > 2 * DEPLOY_IN_ANGLE
        && currentDegrees > 2 * DEPLOY_IN_ANGLE) {

      harvester.setSpinVelocity(SPIN_TARGET_RPS);
      spinStarted = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    harvester.stopDeploy();
    harvester.stopSpin();
  }

  @Override
  public boolean isFinished() {
    double currentDegrees = harvester.getDeployPositionDegrees();

    return Math.abs(currentDegrees - targetDegrees) <= toleranceDegrees;
  }
}
