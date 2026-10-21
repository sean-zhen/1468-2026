package frc.robot.commands;

import static frc.robot.Constants.Harvester.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.HarvesterSubsystem;

public class HarvesterAgitate {

  public static Command create(HarvesterSubsystem harvester) {

    // Define your sequence of target positions
    double[] positions = {70, 20, 70, 20, 0, 20, 0};

    // Build a sequence of commands
    Command[] sequence = new Command[positions.length * 2]; // each pos + wait

    for (int i = 0; i < positions.length; i++) {
      // Fix for lambda: create a final copy of the position
      final double pos = positions[i];

      sequence[i * 2] =
          Commands.runOnce(
              () -> harvester.setHarvDeployMagicMoPos(pos * DEPLOY_DEGREES_TO_ROTATIONS),
              harvester);
      sequence[i * 2 + 1] = Commands.waitSeconds(0.5); // delay between positions
    }

    // Wrap the sequence in a repeating command
    return Commands.repeatingSequence(sequence);
  }
}
