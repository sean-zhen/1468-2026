package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Rollers;

public class RollerSubsystem extends SubsystemBase {

  private final TalonFX ltMotor = new TalonFX(Rollers.LEFT_ROLLER_MOTOR_ID);
  private final TalonFX rtMotor = new TalonFX(Rollers.RIGHT_ROLLER_MOTOR_ID);

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  private final com.ctre.phoenix6.StatusSignal<Angle> ltRollerVeloSignal;
  private final com.ctre.phoenix6.StatusSignal<Angle> rtRollerVeloSignal;

  // Shuffleboard
  private final ShuffleboardTab rollerTab = Shuffleboard.getTab("Rollers");

  private final GenericEntry ltRollerTempEntry;
  private final GenericEntry rtRollerTempEntry;
  private final GenericEntry ltRollerCanOkEntry;
  private final GenericEntry rtRollerCanOkEntry;
  private final GenericEntry ltVeloEntry;

  public RollerSubsystem() {
    configureMotor(ltMotor);
    configureMotor(rtMotor);

    ltRollerVeloSignal = ltMotor.getPosition();
    rtRollerVeloSignal = rtMotor.getPosition();

    // Shuffleboard
    ltVeloEntry =
        rollerTab
            .add("Left Vel (RPS)", 0.0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 0)
            .withSize(2, 1)
            .getEntry();

    var canTab = Shuffleboard.getTab("CAN Status");
    ltRollerCanOkEntry =
        canTab
            .add("Left Roller CAN OK", ltRollerVeloSignal.getStatus().isOK())
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(3, 5)
            .withSize(1, 1)
            .getEntry();

    rtRollerCanOkEntry =
        canTab
            .add("Right Roller CAN OK", rtRollerVeloSignal.getStatus().isOK())
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(4, 5)
            .withSize(1, 1)
            .getEntry();
    ltRollerTempEntry =
        canTab
            .add("Left Roller Temp", ltMotor.getDeviceTemp().getValueAsDouble())
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(3, 5)
            .withSize(1, 1)
            .getEntry();

    rtRollerTempEntry =
        canTab
            .add("Right Roller Temp", rtMotor.getDeviceTemp().getValueAsDouble())
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(4, 5)
            .withSize(1, 1)
            .getEntry();
  }

  private void configureMotor(TalonFX motor) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = Rollers.kP;
    slot0.kI = Rollers.kI;
    slot0.kD = Rollers.kD;
    slot0.kV = Rollers.kV;
    config.Slot0 = slot0;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motor.getConfigurator().apply(config);
    motor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {
    ltVeloEntry.setDouble(getLeftVelocity());
  }

  // -----------------------------
  // Main motor command
  // -----------------------------
  /** Sets roller speed. Positive = normal intake, Negative = reverse/outtake. */
  public void setVelocity(double rps) {
    double leftVel = rps * Rollers.GEAR_RATIO;
    double rightVel = -leftVel; // opposite direction motor

    ltMotor.setControl(velocityRequest.withVelocity(leftVel));
    rtMotor.setControl(velocityRequest.withVelocity(rightVel));
  }

  public void stop() {
    ltMotor.stopMotor();
    rtMotor.stopMotor();
  }

  // -----------------------------
  // Monitoring
  // -----------------------------
  public double getLeftVelocity() {
    return ltMotor.getVelocity().getValueAsDouble() / Rollers.GEAR_RATIO;
  }

  public void setCoastMode() {
    ltMotor.setNeutralMode(NeutralModeValue.Coast);
    rtMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  public void setBrakeMode() {
    ltMotor.setNeutralMode(NeutralModeValue.Brake);
    rtMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public boolean isRollerConnected() {
    // Uses the checks we already built using getStatus().isOK()
    return ltRollerVeloSignal.getStatus().isOK() && rtRollerVeloSignal.getStatus().isOK();
  }
}
