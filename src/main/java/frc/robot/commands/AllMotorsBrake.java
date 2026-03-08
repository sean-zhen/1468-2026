package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drive.Drive;

public class AllMotorsBrake extends Command {
  private final HarvesterSubsystem harvester;
  private final ShooterSubsystem shooter;
  private final IndexerSubsystem indexer;
  private final Drive drive;

  public AllMotorsBrake(
      HarvesterSubsystem harvester,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      Drive drive) {
    this.harvester = harvester;
    this.shooter = shooter;
    this.indexer = indexer;
    this.drive = drive;

    addRequirements(harvester, shooter, indexer, drive);
  }

  @Override
  public void initialize() {

    harvester.setBrakeMode();
    shooter.setBrakeMode();
    indexer.setBrakeMode();
    drive.setBrake();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
