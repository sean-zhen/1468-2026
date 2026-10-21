package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Rollers;
import frc.robot.subsystems.RollerSubsystem;

public class Roll extends Command {
  private final RollerSubsystem roller;

  public Roll(RollerSubsystem roller) {
    this.roller = roller;
    addRequirements(roller);
  }

  @Override
  public void execute() {
    roller.setVelocity(Rollers.NORMAL_RPS); // was 30 RPS
  }

  @Override
  public void end(boolean interrupted) {
    roller.stop();
  }

  @Override
  public boolean isFinished() {
    return false; // Runs until interrupted
  }
}
