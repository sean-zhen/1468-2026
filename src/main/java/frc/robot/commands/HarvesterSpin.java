package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HarvesterSubsystem;

public class HarvesterSpin extends Command {
  private final HarvesterSubsystem harvester;
  private final double directionMultiplier;

  /**
   * @param harvester The subsystem to use.
   * @param reverse If true, spins backwards. If false, spins forwards.
   */
  public HarvesterSpin(HarvesterSubsystem harvester, boolean reverse) {
    this.harvester = harvester;
    // If reverse is true, use -1.0; otherwise use 1.0
    this.directionMultiplier = reverse ? -1.0 : 1.0;

    addRequirements(harvester);
  }

  @Override
  public void execute() {
    // Apply the multiplier to the constant defined in your Constants file
    harvester.setSpinVelocity(Constants.Harvester.SPIN_TARGET_RPS * directionMultiplier);
  }

  @Override
  public void end(boolean interrupted) {
    harvester.stopSpin();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
