package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Kicker;

public class KickerSubsystem extends SubsystemBase {

  private final TalonFX kickerMotor = new TalonFX(Kicker.KICKER_MOTOR_ID);

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  private final com.ctre.phoenix6.StatusSignal<Angle> kickerVeloSignal;

  public KickerSubsystem() {
    // Configure motor
    var config = new com.ctre.phoenix6.configs.TalonFXConfiguration();

    var slot0Configs = new Slot0Configs();
    slot0Configs.kV = Kicker.kV; // Feedforward (Volts per RPS)
    slot0Configs.kP = Kicker.kP; // Proportional gain
    slot0Configs.kI = Kicker.kI; // Integral gain
    slot0Configs.kD = Kicker.kD; // Derivative gain

    config.Slot0 = slot0Configs;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    kickerMotor.getConfigurator().apply(config);

    // Brake mode for kicker
    kickerMotor.setNeutralMode(NeutralModeValue.Brake);

    kickerVeloSignal = kickerMotor.getPosition();
  }

  @Override
  public void periodic() {
    log();
  }

  /**
   * Set the kickwheel velocity in rotations per second (RPS) at the output (wheel) shaft.
   *
   * @param velocityRPS Desired velocity in RPS at the wheel
   */
  public void setVelocity(double velocityRPS) {
    // Convert wheel RPS to motor RPS using gear reduction
    double motorRPS = velocityRPS * Kicker.KICKER_GEAR_RATIO;
    kickerMotor.setControl(velocityRequest.withVelocity(motorRPS));
  }

  public boolean isAtVelocity() {
    double currentVelocity = kickerMotor.getVelocity().getValueAsDouble();
    return Math.abs(currentVelocity - Kicker.KICKER_TARGET_RPS) < Kicker.VELOCITY_TOLERANCE_RPS;
  }

  public void stop() {
    kickerMotor.stopMotor();
  }

  public void log() {
    SmartDashboard.putNumber(
        "Kicker Velo (RPS)",
        kickerMotor.getVelocity().getValueAsDouble() / Kicker.KICKER_GEAR_RATIO);
    SmartDashboard.putNumber("Kicker Temp", kickerMotor.getDeviceTemp().getValueAsDouble());

    boolean kickerOK = kickerVeloSignal.getStatus().isOK();
    SmartDashboard.putBoolean("Kicker CAN OK", kickerOK);
  }

  public boolean isKickerConnected() {
    // Uses the checks we already built using getStatus().isOK()
    return kickerVeloSignal.getStatus().isOK();
  }
}
