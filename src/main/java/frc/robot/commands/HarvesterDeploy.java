package frc.robot.commands;

import static frc.robot.Constants.Harvester.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HarvesterSubsystem;

public class HarvesterDeploy extends Command {
  private final HarvesterSubsystem harvester;
  private final double targetPositionDegrees;
  private final double toleranceDegrees;
  private int counter = 0;

  public HarvesterDeploy(
      HarvesterSubsystem harvester, double targetPositionDegrees, double toleranceDegrees) {
    this.harvester = harvester;
    this.targetPositionDegrees = targetPositionDegrees;
    this.toleranceDegrees = toleranceDegrees;
    addRequirements(harvester);
  }

  @Override
  public void initialize() {
    // Set the target position for Motion Magic
    harvester.setHarvDeployMagicMoPos(targetPositionDegrees * DEPLOY_DEGREES_TO_ROTATIONS);
    // Only spin while harv deployed out
    if (targetPositionDegrees > 2 * DEPLOY_IN_ANGLE) harvester.setSpinVelocity(SPIN_TARGET_RPS);
    else harvester.stopSpin();
    counter = 0;
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    harvester.stopDeploy();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
