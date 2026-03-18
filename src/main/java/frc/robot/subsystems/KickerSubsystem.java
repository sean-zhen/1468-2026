package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Kicker;

public class KickerSubsystem extends SubsystemBase {

  private final TalonFX kickerMotor1 = new TalonFX(Kicker.KICKER1_MOTOR_ID);
  private final TalonFX kickerMotor2 = new TalonFX(Kicker.KICKER2_MOTOR_ID);

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  // Signals for both motors
  private final com.ctre.phoenix6.StatusSignal<AngularVelocity> velocitySignal1;
  private final com.ctre.phoenix6.StatusSignal<AngularVelocity> velocitySignal2;

  // Shuffleboard
  private final ShuffleboardTab kickTab = Shuffleboard.getTab("Kicker");
  private final GenericEntry veloEntry1;
  private final GenericEntry veloEntry2;
  private final GenericEntry tempEntry1;
  private final GenericEntry tempEntry2;
  private final GenericEntry canOkEntry1;
  private final GenericEntry canOkEntry2;

  public KickerSubsystem() {
    configureMotor(kickerMotor1);
    configureMotor(kickerMotor2);

    velocitySignal1 = kickerMotor1.getVelocity();
    velocitySignal2 = kickerMotor2.getVelocity();

    // Shuffleboard widgets
    veloEntry1 =
        kickTab
            .add(
                "Kicker1 Velo (RPS)", velocitySignal1.getValueAsDouble() / Kicker.KICKER_GEAR_RATIO)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 0)
            .withSize(2, 1)
            .getEntry();

    veloEntry2 =
        kickTab
            .add(
                "Kicker2 Velo (RPS)", velocitySignal2.getValueAsDouble() / Kicker.KICKER_GEAR_RATIO)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(2, 0)
            .withSize(2, 1)
            .getEntry();

    tempEntry1 =
        kickTab
            .add("Kicker1 Temp (°C)", kickerMotor1.getDeviceTemp().getValueAsDouble())
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 1)
            .withSize(2, 1)
            .getEntry();

    tempEntry2 =
        kickTab
            .add("Kicker2 Temp (°C)", kickerMotor2.getDeviceTemp().getValueAsDouble())
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(2, 1)
            .withSize(2, 1)
            .getEntry();

    var canTab = Shuffleboard.getTab("CAN Status");
    canOkEntry1 =
        canTab
            .add("Kicker1 CAN OK", velocitySignal1.getStatus().isOK())
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(0, 5)
            .withSize(1, 1)
            .getEntry();

    canOkEntry2 =
        canTab
            .add("Kicker2 CAN OK", velocitySignal2.getStatus().isOK())
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(1, 5)
            .withSize(1, 1)
            .getEntry();
  }

  private void configureMotor(TalonFX motor) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    Slot0Configs slot0 = new Slot0Configs();
    slot0.kV = Kicker.kV;
    slot0.kP = Kicker.kP;
    slot0.kI = Kicker.kI;
    slot0.kD = Kicker.kD;
    config.Slot0 = slot0;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motor.getConfigurator().apply(config);
    motor.setNeutralMode(NeutralModeValue.Brake);
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
    double motorRPS = velocityRPS * Kicker.KICKER_GEAR_RATIO;

    // Both motors same direction
    kickerMotor1.setControl(velocityRequest.withVelocity(motorRPS));
    kickerMotor2.setControl(velocityRequest.withVelocity(motorRPS));
  }

  public boolean isAtVelocity() {
    double v1 = velocitySignal1.getValueAsDouble();
    double v2 = velocitySignal2.getValueAsDouble();
    return Math.abs(v1 - Kicker.KICKER_TARGET_RPS) < Kicker.VELOCITY_TOLERANCE_RPS
        && Math.abs(v2 - Kicker.KICKER_TARGET_RPS) < Kicker.VELOCITY_TOLERANCE_RPS;
  }

  public void stop() {
    kickerMotor1.stopMotor();
    kickerMotor2.stopMotor();
  }

  public void log() {
    // Velocities
    veloEntry1.setDouble(velocitySignal1.getValueAsDouble() / Kicker.KICKER_GEAR_RATIO);
    veloEntry2.setDouble(velocitySignal2.getValueAsDouble() / Kicker.KICKER_GEAR_RATIO);

    // Temperatures
    tempEntry1.setDouble(kickerMotor1.getDeviceTemp().getValueAsDouble());
    tempEntry2.setDouble(kickerMotor2.getDeviceTemp().getValueAsDouble());

    // CAN status
    canOkEntry1.setBoolean(velocitySignal1.getStatus().isOK());
    canOkEntry2.setBoolean(velocitySignal2.getStatus().isOK());
  }

  public boolean isKickerConnected() {
    return velocitySignal1.getStatus().isOK() && velocitySignal2.getStatus().isOK();
  }

  public void setCoastMode() {
    kickerMotor1.setNeutralMode(NeutralModeValue.Coast);
    kickerMotor2.setNeutralMode(NeutralModeValue.Coast);
  }

  public void setBrakeMode() {
    kickerMotor1.setNeutralMode(NeutralModeValue.Brake);
    kickerMotor2.setNeutralMode(NeutralModeValue.Brake);
  }
}
