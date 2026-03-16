package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Indexer;
import java.util.Arrays;

public class IndexerSubsystem extends SubsystemBase {

  private final TalonFX motor = new TalonFX(Indexer.MOTOR_ID);

  private final PositionVoltage positionRequest = new PositionVoltage(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  // Phoenix 6 typed signals
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;

  // CAN health
  private final StatusSignal<Angle> indexerPosSignal;

  // Shuffleboard
  private final ShuffleboardTab idxTab = Shuffleboard.getTab("Indexer");
  private final GenericEntry idxPosEntry;
  private final GenericEntry idxVeloEntry;
  private final GenericEntry idxTempEntry;
  private final GenericEntry idxCanOkEntry;

  // Jam detection
  private static final double JAM_CURRENT_THRESHOLD = 20.0;
  private static final double JAM_CLEAR_TIME = 0.4;
  private static final double JAM_REVERSE_OUTPUT = -0.5;

  private boolean clearingJam = false;
  private double jamClearStart = 0;

  // Jam analytics
  private int jamCount = 0;
  private double lastJamTime = 0;
  private double jamRate = 0; // jams per minute
  private double jamPeakCurrent = 0;

  // Jam history buffer (last 10 jam timestamps)
  private final double[] jamHistory = new double[10];
  private int jamHistoryIndex = 0;

  // Dither jam clear state
  private boolean dithering = false;
  private int ditherStep = 0;
  private double ditherStartTime = 0.0;
  private double ditherBasePosition = 0.0;

  // 60 degrees = 1/6 rotation at the indexer output
  private static final double DITHER_AMPLITUDE_ROT = 1.0 / 12.0; // start small

  // Time per move (fast, but safe)
  private static final double DITHER_STEP_TIME = 0.06; // 60 ms per move

  // Auto slowdown
  private static final int JAM_SLOWDOWN_THRESHOLD = 3; // jams in window
  private static final double SLOWDOWN_FACTOR = 0.6; // 60% speed
  private static final double JAM_WINDOW_SECONDS = 10; // lookback window
  private boolean slowed = false;

  // AdvantageScope logging
  private final DoubleLogEntry jamCountLog;
  private final DoubleLogEntry jamRateLog;
  private final DoubleLogEntry jamPeakLog;

  // Shuffleboard entries
  private final GenericEntry jamCountEntry;
  private final GenericEntry jamRateEntry;
  private final GenericEntry jamPeakEntry;
  private final GenericEntry jamResetButton;
  private final GenericEntry jamSeverityColorEntry;
  private final GenericEntry slowdownEntry;

  public IndexerSubsystem() {

    // Motor config
    var config = new TalonFXConfiguration();
    var slot0 = new Slot0Configs();
    slot0.kP = Indexer.kP;
    slot0.kI = Indexer.kI;
    slot0.kD = Indexer.kD;
    slot0.kV = Indexer.kV;
    config.Slot0 = slot0;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    motor.getConfigurator().apply(config);
    motor.setNeutralMode(NeutralModeValue.Brake);

    // Cache signals
    statorCurrent = motor.getStatorCurrent();
    positionSignal = motor.getPosition();
    velocitySignal = motor.getVelocity();
    indexerPosSignal = motor.getPosition(); // for CAN health

    // Update rates
    statorCurrent.setUpdateFrequency(100);
    positionSignal.setUpdateFrequency(50);
    velocitySignal.setUpdateFrequency(50);

    motor.optimizeBusUtilization();

    // Shuffleboard widgets
    idxPosEntry =
        idxTab
            .add(
                "Indxr Pos (deg, output)",
                positionSignal.getValueAsDouble() * 360.0 / Indexer.GEAR_RATIO)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 0)
            .withSize(2, 1)
            .getEntry();

    idxVeloEntry =
        idxTab
            .add("Indxr Velo (RPS, output)", velocitySignal.getValueAsDouble() / Indexer.GEAR_RATIO)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(2, 0)
            .withSize(2, 1)
            .getEntry();

    idxTempEntry =
        idxTab
            .add("Indxr Temp", motor.getDeviceTemp().getValueAsDouble())
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(4, 0)
            .withSize(2, 1)
            .getEntry();

    var canTab = Shuffleboard.getTab("CAN Status");
    idxCanOkEntry =
        canTab
            .add("Indxr CAN OK", indexerPosSignal.getStatus().isOK())
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(2, 5)
            .withSize(1, 1)
            .getEntry();

    // AdvantageScope log entries
    jamCountLog = new DoubleLogEntry(DataLogManager.getLog(), "Indexer/JamCount");
    jamRateLog = new DoubleLogEntry(DataLogManager.getLog(), "Indexer/JamRate");
    jamPeakLog = new DoubleLogEntry(DataLogManager.getLog(), "Indexer/JamPeakCurrent");

    // Shuffleboard widgets
    jamCountEntry =
        idxTab
            .add("Jam Count", jamCount)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 3)
            .withSize(2, 1)
            .getEntry();

    jamRateEntry =
        idxTab
            .add("Jam Rate (per min)", jamRate)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(2, 3)
            .withSize(2, 1)
            .getEntry();

    jamPeakEntry =
        idxTab
            .add("Jam Peak Current", jamPeakCurrent)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(4, 3)
            .withSize(2, 1)
            .getEntry();

    jamSeverityColorEntry =
        idxTab
            .add("Jam Severity", "GREEN")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 4)
            .withSize(2, 1)
            .getEntry();

    slowdownEntry =
        idxTab
            .add("Auto Slowdown Active", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(2, 4)
            .withSize(2, 1)
            .getEntry();

    jamResetButton =
        idxTab
            .add("Reset Jam Stats", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withPosition(6, 3)
            .withSize(2, 1)
            .getEntry();
  }

  @Override
  public void periodic() {

    // Reset button
    if (jamResetButton.getBoolean(false)) {
      jamCount = 0;
      jamRate = 0;
      jamPeakCurrent = 0;
      Arrays.fill(jamHistory, 0);
      slowed = false;

      jamResetButton.setBoolean(false); // auto-reset button
    }

    // -----------------------------
    // Dither Jam Clear Logic
    // -----------------------------
    if (dithering) {
      double elapsed = Timer.getFPGATimestamp() - ditherStartTime;

      // Track peak current during the whole dither event
      jamPeakCurrent = Math.max(jamPeakCurrent, getStatorCurrent());

      if (elapsed >= DITHER_STEP_TIME) {
        ditherStep++;
        ditherStartTime = Timer.getFPGATimestamp();

        switch (ditherStep) {
          case 1:
            // First oscillation: -amp
            motor.setControl(
                positionRequest.withPosition(ditherBasePosition - DITHER_AMPLITUDE_ROT));
            break;

          case 2:
            // Second oscillation: +amp
            motor.setControl(
                positionRequest.withPosition(ditherBasePosition + DITHER_AMPLITUDE_ROT));
            break;

          case 3:
            // Second oscillation: -amp
            motor.setControl(
                positionRequest.withPosition(ditherBasePosition - DITHER_AMPLITUDE_ROT));
            break;

          case 4:
            // Return to base
            motor.setControl(positionRequest.withPosition(ditherBasePosition));
            break;

          default:
            // Done
            dithering = false;
            clearingJam = false;
            break;
        }
      }
    }

    log();
  }

  // -----------------------------
  // Motor control
  // -----------------------------

  public void setCoastMode() {
    motor.setNeutralMode(NeutralModeValue.Coast);
  }

  public void setBrakeMode() {
    motor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setPosition(double rotations) {
    double motorRotations = rotations * Indexer.GEAR_RATIO;
    motor.setControl(positionRequest.withPosition(motorRotations));
  }

  public void setVelocity(double rps) {

    double motorRPS = rps * Indexer.GEAR_RATIO;

    // Auto slowdown logic
    if (shouldSlowDown()) {
      slowed = true;
      motorRPS *= SLOWDOWN_FACTOR;
    } else {
      slowed = false;
    }

    motor.setControl(velocityRequest.withVelocity(motorRPS));
  }

  public void setPercentOutput(double percent) {
    motor.set(percent);
  }

  public double getStatorCurrent() {
    return statorCurrent.getValueAsDouble();
  }

  public double getPosition() {
    return positionSignal.getValueAsDouble();
  }

  public double getVelocity() {
    return velocitySignal.getValueAsDouble();
  }

  // -----------------------------
  // Jam handling
  // -----------------------------

  public boolean isJammed() {
    return getStatorCurrent() > JAM_CURRENT_THRESHOLD && !clearingJam;
  }

  public void startJamClear() {
    // --- Analytics ---
    clearingJam = true;
    double now = Timer.getFPGATimestamp();
    jamClearStart = now;
    jamCount++;

    if (lastJamTime > 0) {
      double dt = now - lastJamTime;
      if (dt > 0) {
        jamRate = 60.0 / dt; // jams per minute
      }
    }
    lastJamTime = now;

    jamPeakCurrent = getStatorCurrent();

    jamHistory[jamHistoryIndex] = now;
    jamHistoryIndex = (jamHistoryIndex + 1) % jamHistory.length;

    jamCountLog.append(jamCount);
    jamRateLog.append(jamRate);
    jamPeakLog.append(jamPeakCurrent);

    // --- Dither clear ---
    dithering = true;
    ditherStep = 0;
    ditherStartTime = now;
    ditherBasePosition = getPosition();

    // First move command (fast position move)
    motor.setControl(positionRequest.withPosition(ditherBasePosition + DITHER_AMPLITUDE_ROT));
  }

  public boolean isDithering() {
    return dithering;
  }

  public boolean isClearingJam() {
    return false;
  }

  private String getJamSeverityColor() {
    if (jamPeakCurrent < 25) return "GREEN";
    if (jamPeakCurrent < 35) return "YELLOW";
    return "RED";
  }

  private boolean shouldSlowDown() {
    double now = Timer.getFPGATimestamp();
    int recent = 0;

    for (double t : jamHistory) {
      if (t > 0 && now - t <= JAM_WINDOW_SECONDS) {
        recent++;
      }
    }

    return recent >= JAM_SLOWDOWN_THRESHOLD;
  }

  public void forceStopJamClear() {
    clearingJam = false;
    motor.stopMotor();
  }

  public void stop() {
    motor.stopMotor();
  }

  public void logJamEvent() {

    double now = Timer.getFPGATimestamp();
    jamClearStart = now;
    jamCount++;

    if (lastJamTime > 0) {
      double dt = now - lastJamTime;
      if (dt > 0) {
        jamRate = 60.0 / dt;
      }
    }

    lastJamTime = now;

    jamPeakCurrent = getStatorCurrent();

    jamHistory[jamHistoryIndex] = now;
    jamHistoryIndex = (jamHistoryIndex + 1) % jamHistory.length;

    jamCountLog.append(jamCount);
    jamRateLog.append(jamRate);
    jamPeakLog.append(jamPeakCurrent);
  }

  // -----------------------------
  // Logging
  // -----------------------------

  public void log() {
    idxPosEntry.setDouble(positionSignal.getValueAsDouble() * 360.0 / Indexer.GEAR_RATIO);
    idxVeloEntry.setDouble(velocitySignal.getValueAsDouble() / Indexer.GEAR_RATIO);
    idxTempEntry.setDouble(motor.getDeviceTemp().getValueAsDouble());
    idxCanOkEntry.setBoolean(indexerPosSignal.getStatus().isOK());

    jamCountEntry.setDouble(jamCount);
    jamRateEntry.setDouble(jamRate);
    jamPeakEntry.setDouble(jamPeakCurrent);
    jamSeverityColorEntry.setString(getJamSeverityColor());
    slowdownEntry.setBoolean(slowed);
  }

  public boolean isIndexerConnected() {
    return indexerPosSignal.getStatus().isOK();
  }
}
