package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drive.Drive;

public class AllMotorsBrake extends Command {
  private final HarvesterSubsystem harvester;
  private final ShooterSubsystem shooter;
  private final RollerSubsystem rollers;
  private final KickerSubsystem kicker;
  private final Drive drive;

  public AllMotorsBrake(
      HarvesterSubsystem harvester,
      ShooterSubsystem shooter,
      RollerSubsystem rollers,
      KickerSubsystem kicker,
      Drive drive) {
    this.harvester = harvester;
    this.shooter = shooter;
    this.rollers = rollers;
    this.kicker = kicker;
    this.drive = drive;

    addRequirements(harvester, shooter, rollers, kicker, drive);
  }

  @Override
  public void initialize() {

    harvester.setBrakeMode();
    shooter.setBrakeMode();
    rollers.setBrakeMode();
    kicker.setBrakeMode();
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
