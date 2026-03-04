package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HarvesterSubsystem;

/**
 * Command that runs the harvester deploy motor in velocity mode and monitors motor current. When
 * the absolute stator current exceeds the provided threshold for a configured number of consecutive
 * scheduler cycles the command stops the motor. Optionally it will zero the deploy encoder when the
 * stop condition is reached (useful for homing to the "in" mechanical stop).
 */
public class HarvesterDeployVelocityStop extends Command {
  private final HarvesterSubsystem harvester;
  private final double velocityRPS;
  private double currentThresholdAmps;
  private int requiredConsecutiveCycles;
  private final boolean zeroEncoderOnStop;
  private double timeoutSeconds; // <=0 means no timeout
  private final boolean useShuffleboardTuners;

  private int consecutiveCount = 0;

  /**
   * @param harvester the HarvesterSubsystem
   * @param velocityRPS velocity to run the deploy mechanism at (RPS, output shaft). Positive values
   *     should move the mechanism out, negative values move in (match existing project
   *     conventions).
   * @param currentThresholdAmps Amps threshold to consider as a stall/hit the stop
   * @param requiredConsecutiveCycles number of consecutive scheduler cycles current must exceed the
   *     threshold before stopping (helps debounce spikes)
   * @param zeroEncoderOnStop if true, call zeroDeployEncoder() after stopping (for homing)
   */
  public HarvesterDeployVelocityStop(
      HarvesterSubsystem harvester,
      double velocityRPS,
      double currentThresholdAmps,
      int requiredConsecutiveCycles,
      boolean zeroEncoderOnStop,
      double timeoutSeconds,
      boolean useShuffleboardTuners) {
    this.harvester = harvester;
    this.velocityRPS = velocityRPS;
    this.currentThresholdAmps = currentThresholdAmps;
    this.requiredConsecutiveCycles = Math.max(1, requiredConsecutiveCycles);
    this.zeroEncoderOnStop = zeroEncoderOnStop;
    this.timeoutSeconds = timeoutSeconds;
    this.useShuffleboardTuners = useShuffleboardTuners;

    addRequirements(harvester);
  }

  /**
   * Convenience constructor: callers only provide the subsystem and velocity. Other parameters are
   * pulled from Constants.Harvester defaults.
   */
  public HarvesterDeployVelocityStop(HarvesterSubsystem harvester, double velocityRPS) {
    this(
        harvester,
        velocityRPS,
        Constants.Harvester.DEPLOY_HOMING_CURRENT_AMPS,
        Constants.Harvester.DEPLOY_HOMING_CONSECUTIVE_CYCLES,
        Constants.Harvester.DEPLOY_HOMING_ZERO_ON_STOP,
        Constants.Harvester.DEPLOY_HOMING_TIMEOUT_S,
        true);
  }

  @Override
  public void initialize() {
    consecutiveCount = 0;
    startTime = Timer.getFPGATimestamp();
    // Clear any previous endstop flag
    harvester.setDeployEndstopHit(false);
    // If configured to use Shuffleboard tuners, read them now (safe getters in subsystem)
    if (useShuffleboardTuners) {
      currentThresholdAmps = harvester.getDeployHomingCurrentAmps();
      requiredConsecutiveCycles = harvester.getDeployHomingConsecutiveCycles();
      timeoutSeconds = harvester.getDeployHomingTimeoutS();
      // Note: we don't override velocityRPS here; that's still set by the caller.
    }

    // Switch to velocity control (RPS at mechanism output)
    harvester.setDeployVelocity(velocityRPS);
  }

  @Override
  public void execute() {
    double current = harvester.getDeployStatorCurrent();
    if (Math.abs(current) >= Math.abs(currentThresholdAmps)) {
      consecutiveCount++;
    } else {
      consecutiveCount = 0;
    }
  }

  @Override
  public void end(boolean interrupted) {
    harvester.stopDeploy();
    // If we finished normally because we detected the endstop via current, mark the flag and
    // optionally zero the encoder. If interrupted, do not zero.
    if (!interrupted) {
      if (consecutiveCount >= requiredConsecutiveCycles) {
        harvester.setDeployEndstopHit(true);
        if (zeroEncoderOnStop) harvester.zeroDeployEncoder();
      }
    }
  }

  @Override
  public boolean isFinished() {
    if (consecutiveCount >= requiredConsecutiveCycles) return true;
    if (timeoutSeconds > 0 && (Timer.getFPGATimestamp() - startTime) >= timeoutSeconds) return true;
    return false;
  }

  private double startTime = 0.0;
}
