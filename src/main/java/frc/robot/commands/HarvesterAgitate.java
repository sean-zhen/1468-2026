package frc.robot.commands;

import static frc.robot.Constants.Harvester.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.HarvesterSubsystem;

public class HarvesterAgitate {

  public static Command create(HarvesterSubsystem harvester) {
    return Commands.repeatingSequence(
        Commands.runOnce(
            () -> harvester.setHarvDeployMagicMoPos(70 * DEPLOY_DEGREES_TO_ROTATIONS), harvester),
        Commands.waitSeconds(0.5),
        Commands.runOnce(
            () -> harvester.setHarvDeployMagicMoPos(20 * DEPLOY_DEGREES_TO_ROTATIONS), harvester),
        Commands.waitSeconds(0.5));
  }
}
