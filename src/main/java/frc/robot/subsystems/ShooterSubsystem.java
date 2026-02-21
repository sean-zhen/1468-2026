package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooter;
import java.util.List;

public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX flywheelLead = new TalonFX(Shooter.FLYWHEEL_LEAD_ID);
  private final TalonFX flywheelFollower = new TalonFX(Shooter.FLYWHEEL_FOLLOWER_ID);
  private final TalonFX hoodMotor = new TalonFX(Shooter.HOOD_MOTOR_ID);
  private final TalonFX turretMotor = new TalonFX(Shooter.TURRET_MOTOR_ID);

  private final VelocityVoltage leadRequest = new VelocityVoltage(0);
  private final PositionVoltage hoodPositionRequest = new PositionVoltage(0);
  private final VelocityVoltage hoodVelocityRequest = new VelocityVoltage(0);
  private final VelocityVoltage turretVelocityRequest = new VelocityVoltage(0);

  // --- Control Requests ---
  private final VelocityVoltage m_velocityReq = new VelocityVoltage(0);
  private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);

  private double targetFlyWheelVeloRPS = 0.0;
  private double targetTurretPositionRot = 0.0;
  private double targetHoodPositionRot = 0.0;

  // 6 Interpolation Maps
  private final InterpolatingDoubleTreeMap flyAlliance = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap hoodAlliance = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap flyNeutral = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap hoodNeutral = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap flyOpposition = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap hoodOpposition = new InterpolatingDoubleTreeMap();

  private double targetTurretRot = 0;

  private final com.ctre.phoenix6.StatusSignal<Angle> turretPositionSignal;
  private final com.ctre.phoenix6.StatusSignal<AngularVelocity> flywheelVelocitySignal;
  private final com.ctre.phoenix6.StatusSignal<Angle> hoodPositionSignal;

  public ShooterSubsystem() {
    // // Flywheel config
    // var flywheelConfig = new TalonFXConfiguration();
    // var slot0Configs = new Slot0Configs();
    // slot0Configs.kV = Shooter.SHOOT_kV;
    // slot0Configs.kA = Shooter.SHOOT_kA;
    // slot0Configs.kP = Shooter.SHOOT_kP;
    // slot0Configs.kI = Shooter.SHOOT_kI;
    // slot0Configs.kD = Shooter.SHOOT_kD;
    // flywheelConfig.Slot0 = slot0Configs;
    // flywheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    // flywheelLead.getConfigurator().apply(flywheelConfig);
    // flywheelLead.setNeutralMode(NeutralModeValue.Coast);

    // // Follower config
    // flywheelFollower.setControl(
    //     new Follower(flywheelLead.getDeviceID(), MotorAlignmentValue.Opposed));
    // flywheelFollower.setNeutralMode(NeutralModeValue.Coast);

    // // Hood config (positional PID or velocity, brake mode, soft stops)
    // var hoodConfig = new TalonFXConfiguration();
    // var hoodSlot0 = new Slot0Configs();
    // hoodSlot0.kP = Shooter.HOOD_kP;
    // hoodSlot0.kI = Shooter.HOOD_kI;
    // hoodSlot0.kD = Shooter.HOOD_kD;
    // hoodSlot0.kV = Shooter.HOOD_kV;
    // hoodConfig.Slot0 = hoodSlot0;
    // hoodConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    // hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Shooter.HOOD_TOP_SOFT_LIMIT_ROT;
    // hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
    // Shooter.HOOD_BOTTOM_SOFT_LIMIT_ROT;
    // hoodMotor.getConfigurator().apply(hoodConfig);
    // hoodMotor.setNeutralMode(NeutralModeValue.Brake);

    // // Turret config (velocity, brake mode, soft stops)
    // var turretConfig = new TalonFXConfiguration();
    // var turretSlot0 = new Slot0Configs();
    // turretSlot0.kP = Shooter.TURRET_kP;
    // turretSlot0.kI = Shooter.TURRET_kI;
    // turretSlot0.kD = Shooter.TURRET_kD;
    // turretSlot0.kV = Shooter.TURRET_kV;
    // turretConfig.Slot0 = turretSlot0;
    // turretConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    // turretConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // turretConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
    //     Shooter.TURRET_RIGHT_SOFT_LIMIT_ROT;
    // turretConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // turretConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
    // Shooter.TURRET_LEFT_SOFT_LIMIT_ROT;
    // turretMotor.getConfigurator().apply(turretConfig);
    // turretMotor.setNeutralMode(NeutralModeValue.Brake);

    configureMotors();
    setupTables();

    turretPositionSignal = turretMotor.getPosition();
    flywheelVelocitySignal = flywheelLead.getVelocity();
    hoodPositionSignal = hoodMotor.getPosition();
  }

  @Override
  public void periodic() {
    log();
  }

  private void configureMotors() {
    // --- Lead Flywheel (Velocity PID) ---
    TalonFXConfiguration leadConfigs = new TalonFXConfiguration();
    leadConfigs.Slot0.kP = 0.11;
    leadConfigs.Slot0.kV = 0.12;
    leadConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    leadConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    flywheelLead.getConfigurator().apply(leadConfigs);

    // --- Follower Flywheel (Facing Lead) ---
    // Opposed alignment: The follower will invert its direction relative to the lead
    // because they face each other.
    // flywheelFollower.setControl(new Follower(flywheelLead.getDeviceID(), true));

    // Follower config
    flywheelFollower.setControl(
        new Follower(flywheelLead.getDeviceID(), MotorAlignmentValue.Opposed));
    flywheelFollower.setNeutralMode(NeutralModeValue.Coast);

    // Ensure follower is also in Coast mode
    TalonFXConfiguration followerConfigs = new TalonFXConfiguration();
    followerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flywheelFollower.getConfigurator().apply(followerConfigs);

    // --- Turret (Motion Magic - 11:1 Ratio) ---
    TalonFXConfiguration turretConfigs = new TalonFXConfiguration();
    turretConfigs.Feedback.SensorToMechanismRatio = 11.0;

    // TODO: TA - tune these constants
    turretConfigs.Slot0.kS = 0.25; // 0.6;  0.25; // Voltage to overcome internal/belt friction
    turretConfigs.Slot0.kV = 0.12; // Base speed feedforward
    turretConfigs.Slot0.kP = 200; // 50.0; // High P for "snappy" reaction to inputs
    //    turretConfigs.Slot0.kP = 1.0;
    // turretConfigs.Slot0.kI = 0.12;

    // 2.0 Rotations/Sec = 720 degrees/sec.
    turretConfigs.MotionMagic.MotionMagicCruiseVelocity = 1.0; // 0.5;  2.0;
    turretConfigs.MotionMagic.MotionMagicAcceleration =
        3.0; // 1.0;  4.0; // 0.5s to reach top speed
    turretConfigs.MotionMagic.MotionMagicJerk = 0.0; // Optional: smooths the "kick"  - WAS 40
    turretConfigs.MotorOutput.DutyCycleNeutralDeadband = 0.001; // Reach the exact end point

    // turretConfigs.MotionMagic.MotionMagicCruiseVelocity = 60;
    // turretConfigs.MotionMagic.MotionMagicAcceleration = 120;
    turretConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    turretConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    turretConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Shooter.TURRET_RIGHT_SOFT_LIMIT_ROT;
    turretConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    turretConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Shooter.TURRET_LEFT_SOFT_LIMIT_ROT;
    turretMotor.getConfigurator().apply(turretConfigs);
    turretMotor.setNeutralMode(NeutralModeValue.Brake);

    // --- Hood (Motion Magic) ---
    TalonFXConfiguration hoodConfigs = new TalonFXConfiguration();

    hoodConfigs.Feedback.SensorToMechanismRatio = 15.0; // TODO TA: Get real reduction

    // hoodConfigs.Slot0.kP = 1.5;
    // hoodConfigs.Slot0.kV = 0.12;
    // hoodConfigs.MotionMagic.MotionMagicCruiseVelocity = 40;
    // hoodConfigs.MotionMagic.MotionMagicAcceleration = 80;

    hoodConfigs.Slot0.kS = 0.25; // Voltage to overcome friction
    hoodConfigs.Slot0.kV = 0.12; // Theoretical kV for Kraken
    hoodConfigs.Slot0.kP = 2.4; // Higher P for precision positioning
    hoodConfigs.MotionMagic.MotionMagicCruiseVelocity = 1.0; // 1 RPS
    hoodConfigs.MotionMagic.MotionMagicAcceleration = 2.0;

    hoodConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    hoodConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Keep Brake for hood
    hoodConfigs.MotorOutput.DutyCycleNeutralDeadband = 0.001; // Help reach the final point

    hoodConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    hoodConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Shooter.HOOD_TOP_SOFT_LIMIT_ROT;
    hoodConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    hoodConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Shooter.HOOD_BOTTOM_SOFT_LIMIT_ROT;
    hoodMotor.getConfigurator().apply(hoodConfigs);
    hoodMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  // Flywheel velocity (RPS at output) - this is for testing, using joystick Z axis for velo
  public void setFlywheel(double velocityRPS) {
    velocityRPS = velocityRPS * 100; // Temporary scaling for testing
    this.targetFlyWheelVeloRPS = velocityRPS;
    flywheelLead.setControl(leadRequest.withVelocity(velocityRPS));
  }

  public void setShooterParams(double distance, String zone) {
    double flyRPS =
        zone.equals("Alliance")
            ? flyAlliance.get(distance)
            : zone.equals("Neutral") ? flyNeutral.get(distance) : flyOpposition.get(distance);
    double hoodRot =
        zone.equals("Alliance")
            ? hoodAlliance.get(distance)
            : zone.equals("Neutral") ? hoodNeutral.get(distance) : hoodOpposition.get(distance);

    // flywheelLead.setControl(new VelocityVoltage(flyRPS));
    // hoodMotor.setControl(new PositionVoltage(hoodRot * Shooter.HOOD_GEAR_RATIO));
    flywheelLead.setControl(m_velocityReq.withVelocity(flyRPS));
    hoodMotor.setControl(m_mmReq.withPosition(hoodRot));
  }

  public double getFlywheelVeloRPS() {
    return flywheelLead.getVelocity().getValueAsDouble();
  }

  public boolean isFLywheelAtVelocity() {
    double currentVelocity = flywheelLead.getVelocity().getValueAsDouble();
    // TAKE A LOOK TO SEE IF SCALING IS NEEDED HERE
    return Math.abs(currentVelocity - targetFlyWheelVeloRPS) < (Shooter.VELOCITY_TOLERANCE_RPS);
  }

  // Hood position (rotations at output)
  public void setHoodPosition(double rotations) {
    this.targetHoodPositionRot = rotations;
    // double motorRotations = rotations * Shooter.HOOD_GEAR_RATIO;
    // hoodMotor.setControl(hoodPositionRequest.withPosition(motorRotations));
    hoodMotor.setControl(m_mmReq.withPosition(rotations));
  }

  // // Hood velocity (RPS at output)
  // public void setHoodVelocity(double rps) {
  //   double motorRPS = rps * Shooter.HOOD_GEAR_RATIO;
  //   hoodMotor.setControl(hoodVelocityRequest.withVelocity(motorRPS));
  // }

  // Turret position getter
  public double getHoodPosition() {
    return hoodMotor.getPosition().getValueAsDouble() / Shooter.TURRET_GEAR_RATIO;
  }

  public boolean isHoodAtPosition() {
    double currentPosition = getHoodPosition();
    double toleranceRot = Math.toRadians(Shooter.HOOD_TRACKING_TOLERANCE_DEG) / (2 * Math.PI);
    return Math.abs(currentPosition - targetHoodPositionRot) < toleranceRot;
  }

  public void setTurretPosition(double rotations) {
    // SAFETY: Clamp to stop at +/- 135 degrees [Hardware Limit Check]
    double clampedRot =
        //        MathUtil.clamp(rotations, -Shooter.TURRET_LIMIT_ROT, Shooter.TURRET_LIMIT_ROT);
        tomClamp(rotations);
    turretMotor.setControl(m_mmReq.withPosition(clampedRot));
  }

  public void setTurretWithFF(double rotations, double feedforwardRPS) {
    double clamped =
        //    MathUtil.clamp(rotations, -Shooter.TURRET_LIMIT_ROT, Shooter.TURRET_LIMIT_ROT);
        tomClamp(rotations);
    this.targetTurretRot = clamped;
    double ffVolts = feedforwardRPS * Shooter.TURRET_kV;

    // turretMotor.setControl(
    //     new PositionVoltage(clamped).withFeedForward(ffVolts));
    turretMotor.setControl(m_mmReq.withPosition(clamped));
  }

  public double tomClamp(double rotations) {
    double clampedRot;
    if (rotations > 1) rotations = rotations % 1;
    if (rotations < -1) rotations = rotations % 1;
    clampedRot = rotations;
    if (rotations > Shooter.TURRET_LIMIT_ROT)
      clampedRot = (0.5 - rotations) * Shooter.TURRET_CLAMP_FACTOR;
    if (rotations < -Shooter.TURRET_LIMIT_ROT)
      clampedRot = (-0.5 - rotations) * Shooter.TURRET_CLAMP_FACTOR;

    return clampedRot;
  }
  ;

  // Turret velocity (RPS at output)
  public void setTurretVelocity(double rps) {
    double motorRPS = rps * Shooter.TURRET_GEAR_RATIO;
    turretMotor.setControl(turretVelocityRequest.withVelocity(motorRPS));
  }

  // Turret position getter
  public double getTurretPosition() {
    return turretMotor.getPosition().getValueAsDouble() / Shooter.TURRET_GEAR_RATIO;
  }

  public boolean isTurretAtPosition() {
    double currentPosition = getTurretPosition();
    double toleranceRot = Math.toRadians(Shooter.TURRET_TRACKING_TOLERANCE_DEG) / (2 * Math.PI);
    return Math.abs(currentPosition - targetTurretPositionRot) < toleranceRot;
  }

  public void stop() {
    flywheelLead.stopMotor();
    flywheelFollower.stopMotor();
    hoodMotor.stopMotor();
    turretMotor.stopMotor();
  }

  public void log() {

    // Display green/red boolean indicators for drive team
    SmartDashboard.putBoolean("Shtr Is At Spd", isFLywheelAtVelocity());
    SmartDashboard.putBoolean("Trrt Is At Pos", isTurretAtPosition());
    SmartDashboard.putBoolean("Hood Is At Pos", isHoodAtPosition());

    // Check CAN health for the dashboard
    // Since these signals are refreshed in the Command via waitForAll,
    // getStatus() returns the StatusCode enum
    boolean shooterOK = flywheelVelocitySignal.getStatus().isOK();
    boolean turretOK = turretPositionSignal.getStatus().isOK();
    boolean hoodOK = hoodPositionSignal.getStatus().isOK();

    SmartDashboard.putBoolean("Shtr CAN OK", shooterOK && turretOK && hoodOK);
    SmartDashboard.putBoolean("FlyWh CAN OK", shooterOK);
    SmartDashboard.putBoolean("Trrt CAN OK", turretOK);
    SmartDashboard.putBoolean("Hood CAN OK", hoodOK);

    // Flywheel
    SmartDashboard.putNumber("Shtr Lead Velo (RPS)", flywheelLead.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber(
        "Shtr Flwr Velo (RPS)", flywheelFollower.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shtr Lead Cmd", flywheelLead.get());
    SmartDashboard.putNumber("Shtr Flwr Cmd", flywheelFollower.get());
    SmartDashboard.putNumber("Shtr Lead Temp", flywheelLead.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("Shtr Flwr Temp", flywheelFollower.getDeviceTemp().getValueAsDouble());

    // Hood
    SmartDashboard.putNumber(
        "Shtr Hood Pos (rot, output)",
        hoodMotor.getPosition().getValueAsDouble() / Shooter.HOOD_GEAR_RATIO);
    SmartDashboard.putNumber(
        "Shtr Hood Velo (RPS, output)",
        hoodMotor.getVelocity().getValueAsDouble() / Shooter.HOOD_GEAR_RATIO);
    SmartDashboard.putNumber("Hood Temp", hoodMotor.getDeviceTemp().getValueAsDouble());

    // Turret
    SmartDashboard.putNumber(
        "Shtr Turret Pos (rot, output)",
        turretMotor.getPosition().getValueAsDouble() / Shooter.TURRET_GEAR_RATIO);
    SmartDashboard.putNumber(
        "Shtr Turret Velo (RPS, output)",
        turretMotor.getVelocity().getValueAsDouble() / Shooter.TURRET_GEAR_RATIO);
    SmartDashboard.putNumber("Turret Temp", turretMotor.getDeviceTemp().getValueAsDouble());
  }

  private void setupTables() {
    // Alliance Zone Flywheel and Hood Tables // TODO TA: Must empirically obtain values
    // parameters are distance to Hub in meters and RPS velocity of the flywheel
    flyAlliance.put(0.0, 45.0);
    flyAlliance.put(1.0, 45.0); // ABOUT THE SHORTEST SHOT IN OUR ALLIANCE ZONE
    flyAlliance.put(2.0, 50.0);
    flyAlliance.put(4.0, 55.0);
    flyAlliance.put(6.0, 60.0); // ABOUT THE LONGEST SHOT IN OUR ALLIANCE ZONE
    flyAlliance.put(8.0, 60.0);

    // parameters are distance to Hub in meters and Hood angle in rotations
    hoodAlliance.put(0.0, 0.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodAlliance.put(1.0, 0.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodAlliance.put(2.0, 5.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodAlliance.put(4.0, 10.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodAlliance.put(6.0, 15.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodAlliance.put(8.0, 20.0 / Shooter.HOOD_DEG_PER_ROTATION);

    // Neutral Zone Flywheel and Hood Tables // TODO TA: Must empirically obtain values
    // parameters are distance to Hub in meters and RPS velocity of the flywheel
    flyNeutral.put(0.0, 55.0);
    flyNeutral.put(4.0, 55.0); // ABOUT THE SHORTEST SHOT IN THE NEUTRAL ZONE
    flyNeutral.put(7.0, 60.0);
    flyNeutral.put(10.0, 65.0);
    flyNeutral.put(12.0, 70.0); // ABOUT THE LONGEST SHOT IN THE NEUTRAL ZONE
    flyNeutral.put(15.0, 75.0);

    // parameters are distance to Hub in meters and Hood angle in rotation
    hoodNeutral.put(0.0, 10.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodNeutral.put(4.0, 10.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodNeutral.put(7.0, 15.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodNeutral.put(10.0, 20.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodNeutral.put(12.0, 25.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodNeutral.put(15.0, 30.0 / Shooter.HOOD_DEG_PER_ROTATION);

    // Opposition Zone Flywheel and Hood Tables // TODO TA: Must empirically obtain values
    // parameters are distance to Hub in meters and RPS velocity of the flywheel
    flyOpposition.put(0.0, 85.0);
    flyOpposition.put(10.0, 65.0); // ABOUT THE SHORTEST SHOT IN OPPONENTS ZONE
    flyOpposition.put(12.0, 70.0);
    flyOpposition.put(15.0, 75.0);
    flyOpposition.put(20.0, 80.0);

    // parameters are distance to Hub in meters and Hood angle in rotation
    hoodOpposition.put(0.0, 20.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodOpposition.put(10.0, 20.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodOpposition.put(12.0, 25.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodOpposition.put(15.0, 30.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodOpposition.put(20.0, 35.0 / Shooter.HOOD_DEG_PER_ROTATION);
  }

  public List<BaseStatusSignal> getSignals() {
    // Explicitly define the list type to avoid generic mismatch
    return List.of(
        (BaseStatusSignal) turretPositionSignal,
        (BaseStatusSignal) flywheelVelocitySignal,
        (BaseStatusSignal) hoodPositionSignal);
  }

  public boolean isShooterConnected() {
    // Uses the checks we already built using getStatus().isOK()
    return flywheelVelocitySignal.getStatus().isOK()
        && turretPositionSignal.getStatus().isOK()
        && hoodPositionSignal.getStatus().isOK();
  }
}
