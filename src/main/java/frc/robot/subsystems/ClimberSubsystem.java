package frc.robot.subsystems;

import static frc.robot.Constants.Climber.*;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
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
import frc.robot.Constants.Climber;

public class ClimberSubsystem extends SubsystemBase {

  private final TalonFX leftClimberMotor = new TalonFX(Climber.LEFT_MOTOR_ID);
  // private final TalonFX rightClimberMotor = new TalonFX(Climber.RIGHT_MOTOR_ID);

  private final PositionVoltage positionRequest = new PositionVoltage(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  private final com.ctre.phoenix6.StatusSignal<Angle> climberPositionSignal;
  // Shuffleboard entries
  private final ShuffleboardTab climbTab = Shuffleboard.getTab("Climber");
  private final GenericEntry climberPosEntry;
  private final GenericEntry climberSpeedEntry;
  private final GenericEntry climberTempEntry;
  private final GenericEntry climberTopEntry;
  private final GenericEntry climberBotEntry;
  private final GenericEntry climberCanOkEntry;

  public ClimberSubsystem() {
    // Configure left motor
    TalonFXConfiguration config = new TalonFXConfiguration();
    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = Climber.kP;
    slot0.kI = Climber.kI;
    slot0.kD = Climber.kD;
    slot0.kV = Climber.kV;
    config.Slot0 = slot0;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Climber.FORWARD_SOFT_LIMIT_ROT;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Climber.REVERSE_SOFT_LIMIT_ROT;
    leftClimberMotor.getConfigurator().apply(config);
    leftClimberMotor.setNeutralMode(NeutralModeValue.Brake);

    climberPositionSignal = leftClimberMotor.getPosition();

    /* Configure gear ratio */
    FeedbackConfigs fdb = config.Feedback;
    fdb.SensorToMechanismRatio = 1.0; // 1 rotor rotations per mechanism rotation
    setPosition(HOME_POSITION_ROT); // Start at Zero position }
    // Persistent Shuffleboard widgets
    climberPosEntry =
        climbTab
            .add("Climber Pos (deg)", getEncoderRotations() * 360.0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 0)
            .withSize(2, 1)
            .getEntry();
    climberSpeedEntry =
        climbTab
            .add("Climber Speed", leftClimberMotor.get())
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(2, 0)
            .withSize(2, 1)
            .getEntry();
    climberTempEntry =
        climbTab
            .add("Climber Temp", getLeftClimberTemp())
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(4, 0)
            .withSize(2, 1)
            .getEntry();
    climberTopEntry =
        climbTab
            .add("Climber Is At Top", isAtTop())
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(0, 1)
            .withSize(2, 1)
            .getEntry();
    climberBotEntry =
        climbTab
            .add("Climber Is At Bot", isAtBot())
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(2, 1)
            .withSize(2, 1)
            .getEntry();

    var canTab = Shuffleboard.getTab("CAN Status");
    climberCanOkEntry =
        canTab
            .add("Clmbr CAN OK", climberPositionSignal.getStatus().isOK())
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(4, 5)
            .withSize(1, 1)
            .getEntry();
  }

  @Override
  public void periodic() {
    log();
  }

  public void setCoastMode() {
    leftClimberMotor.setNeutralMode(NeutralModeValue.Coast);
    //    rightClimberMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  public void setBrakeMode() {
    leftClimberMotor.setNeutralMode(NeutralModeValue.Brake);
    //    rightClimberMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setMotorSpeed(double speed) {
    leftClimberMotor.set(speed);
  }

  public void climberUp() {
    leftClimberMotor.set(Climber.UP_SPEED);
  }

  public void climberDown() {
    leftClimberMotor.set(Climber.DOWN_SPEED);
  }

  public void climberStop() {
    leftClimberMotor.set(0);
  }

  public void setPosition(double rotations) {
    leftClimberMotor.setControl(positionRequest.withPosition(rotations));
  }

  public void setVelocity(double rps) {
    leftClimberMotor.setControl(velocityRequest.withVelocity(rps));
  }

  public double getEncoderRotations() {
    return leftClimberMotor.getPosition().getValueAsDouble();
  }

  public double getSmallUpLocation() {
    double temp = getEncoderRotations();
    if ((Climber.FORWARD_SOFT_LIMIT_ROT - temp) > Climber.SMALL_MOVE_ROT)
      return (temp + Climber.SMALL_MOVE_ROT);
    else return (Climber.FORWARD_SOFT_LIMIT_ROT - 0.1);
  }

  public double getSmallDownLocation() {
    double temp = getEncoderRotations();
    if (temp > Climber.SMALL_MOVE_ROT) return (temp - Climber.SMALL_MOVE_ROT);
    else return Climber.HOME_POSITION_ROT;
  }

  public boolean isAtTop() {
    return (leftClimberMotor.get() > 0.1)
        && (getEncoderRotations() > Climber.FORWARD_SOFT_LIMIT_ROT);
  }

  public boolean isAtBot() {
    return (leftClimberMotor.get() < -0.1)
        && (getEncoderRotations() < (Climber.REVERSE_SOFT_LIMIT_ROT + 0.125));
  }

  public void log() {
    // Update persistent entries
    climberPosEntry.setDouble(getEncoderRotations() * 360.0);
    climberSpeedEntry.setDouble(leftClimberMotor.get());
    climberTempEntry.setDouble(getLeftClimberTemp());
    climberTopEntry.setBoolean(isAtTop());
    climberBotEntry.setBoolean(isAtBot());

    boolean climberOK = climberPositionSignal.getStatus().isOK();
    climberCanOkEntry.setBoolean(climberOK);
  }

  public double getLeftClimberTemp() {
    return leftClimberMotor.getDeviceTemp().getValueAsDouble();
  }

  public boolean isClimberConnected() {
    // Uses the checks we already built using getStatus().isOK()
    return climberPositionSignal.getStatus().isOK();
  }
}
