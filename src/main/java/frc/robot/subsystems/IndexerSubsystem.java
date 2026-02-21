package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Indexer;

public class IndexerSubsystem extends SubsystemBase {
  private final TalonFX motor = new TalonFX(Indexer.MOTOR_ID);

  private final PositionVoltage positionRequest = new PositionVoltage(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  private final com.ctre.phoenix6.StatusSignal<Angle> indexerVeloSignal;

  public IndexerSubsystem() {
    var config = new com.ctre.phoenix6.configs.TalonFXConfiguration();
    var slot0 = new Slot0Configs();
    slot0.kP = Indexer.kP;
    slot0.kI = Indexer.kI;
    slot0.kD = Indexer.kD;
    slot0.kV = Indexer.kV;
    config.Slot0 = slot0;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motor.getConfigurator().apply(config);
    motor.setNeutralMode(NeutralModeValue.Brake);

    indexerVeloSignal = motor.getPosition();
  }

  @Override
  public void periodic() {
    log();
  }

  // Position control (rotations at output)
  public void setPosition(double rotations) {
    double motorRotations = rotations * Indexer.GEAR_RATIO;
    motor.setControl(positionRequest.withPosition(motorRotations));
  }

  // Velocity control (RPS at output)
  public void setVelocity(double rps) {
    double motorRPS = rps * Indexer.GEAR_RATIO;
    motor.setControl(velocityRequest.withVelocity(motorRPS));
  }

  public double getPosition() {
    return motor.getPosition().getValueAsDouble(); // Returns rotations at motor shaft
  }

  public void stop() {
    motor.stopMotor();
  }

  public void log() {
    // ── Indexer subsystem page ────────────────────────────────────────────────
    var idxTab = Shuffleboard.getTab("Indexer");

    idxTab
        .add(
            "Indxr Pos (rotations, output)",
            motor.getPosition().getValueAsDouble() / Indexer.GEAR_RATIO)
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(0, 0)
        .withSize(2, 1);
    idxTab
        .add(
            "Indxr Velo (RPS, output)",
            motor.getVelocity().getValueAsDouble() / Indexer.GEAR_RATIO)
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(2, 0)
        .withSize(2, 1);
    idxTab
        .add("Indxr Temp", motor.getDeviceTemp().getValueAsDouble())
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(4, 0)
        .withSize(2, 1);

    // ── CAN Status page ───────────────────────────────────────────────────────
    boolean indexerOK = indexerVeloSignal.getStatus().isOK();
    Shuffleboard.getTab("CAN Status")
        .add("Indxr CAN OK", indexerOK)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(2, 5)
        .withSize(1, 1);
  }

  public boolean isIndexerConnected() {
    // Uses the checks we already built using getStatus().isOK()
    return indexerVeloSignal.getStatus().isOK();
  }
}
