package frc.robot.commands;

import static frc.robot.Constants.Harvester.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HarvesterSubsystem;

public class HarvesterDeploy extends Command {
  private final HarvesterSubsystem harvester;
  private final double targetPositionDegrees;
  private final double toleranceDegrees;
  private int counter = 0;
  private boolean spinStarted = false;

  public HarvesterDeploy(
      HarvesterSubsystem harvester, double targetPositionDegrees, double toleranceDegrees) {
    this.harvester = harvester;
    this.targetPositionDegrees = targetPositionDegrees;
    this.toleranceDegrees = toleranceDegrees;
    addRequirements(harvester);
  }

  @Override
  public void initialize() {
    harvester.setHarvDeployMagicMoPos(targetPositionDegrees * DEPLOY_DEGREES_TO_ROTATIONS);
    spinStarted = false;
  }

  @Override
  public void execute() {

    double currentDegrees = harvester.getDeployPositionDegrees();

    // boolean atTarget =
    //     Math.abs(currentDegrees - targetPositionDegrees) <= toleranceDegrees;

    if (!spinStarted
        && targetPositionDegrees > 2 * DEPLOY_IN_ANGLE
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
    return false;
  }
}
