package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.util.function.DoubleSupplier;

public class Shoot extends Command {
  private final ShooterSubsystem shooter;
  private final DoubleSupplier flywheelSpeed;
  private final DoubleSupplier hoodSpeed;

  /**
   * Creates a Shoot command.
   *
   * @param shooter The subsystem used by this command.
   * @param flywheelSpeed Supplier for flywheel speed (-1.0 to 1.0).
   * @param hoodSpeed Supplier for hood speed (-1.0 to 1.0).
   */
  public Shoot(ShooterSubsystem shooter, DoubleSupplier flywheelSpeed, DoubleSupplier hoodSpeed) {
    this.shooter = shooter;
    this.flywheelSpeed = flywheelSpeed;
    this.hoodSpeed = hoodSpeed;

    // helper to ensure no other command uses the shooter at the same time
    addRequirements(shooter);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Update both flywheel motors and the hood motor constantly
    if (flywheelSpeed.getAsDouble() < 0.2 && flywheelSpeed.getAsDouble() > -0.2) {
      shooter.setFlywheel(0.0);
    }

    shooter.setFlywheel(flywheelSpeed.getAsDouble());
    shooter.setHood(hoodSpeed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // Runs until interrupted (e.g. button released)
  }
}
