package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Indexer;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexerSpin extends Command {
  private final IndexerSubsystem indexer;
  private final double directionMultiplier;

  public IndexerSpin(IndexerSubsystem indexer, boolean reverse) {
    this.indexer = indexer;
    this.directionMultiplier = reverse ? -1.0 : 1.0;

    addRequirements(indexer);
  }

  @Override
  public void execute() {
    indexer.setVelocity(Indexer.TARGET_RPS * directionMultiplier);
  }

  @Override
  public void end(boolean interrupted) {

    // Find nearest position
    double currentPosition = indexer.getPosition();
    double modPosition = currentPosition % 9;
    double nearest = Indexer.POSITIONS[0];
    double minDiff = Math.abs(modPosition - nearest);

    for (double pos : Indexer.POSITIONS) {
      double diff = Math.abs(currentPosition - pos);
      if (diff < minDiff) {
        minDiff = diff;
        nearest = pos;
      }
    }
    indexer.setPosition(currentPosition - modPosition);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
