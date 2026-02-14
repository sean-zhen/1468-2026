package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooter;
import org.photonvision.PhotonCamera;

public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX flywheelLead = new TalonFX(Shooter.FLYWHEEL_LEAD_ID);
  private final TalonFX flywheelFollower = new TalonFX(Shooter.FLYWHEEL_FOLLOWER_ID);
  private final TalonFX hoodMotor = new TalonFX(Shooter.HOOD_MOTOR_ID);
  private final TalonFX turretMotor = new TalonFX(Shooter.TURRET_MOTOR_ID);

  private final PhotonCamera camera = new PhotonCamera(Shooter.CAMERA_NAME);

  private final VelocityVoltage leadRequest = new VelocityVoltage(0);
  private final PositionVoltage hoodPositionRequest = new PositionVoltage(0);
  private final VelocityVoltage hoodVelocityRequest = new VelocityVoltage(0);
  private final VelocityVoltage turretVelocityRequest = new VelocityVoltage(0);

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

  public ShooterSubsystem() {
    // Flywheel config
    var flywheelConfig = new TalonFXConfiguration();
    var slot0Configs = new Slot0Configs();
    slot0Configs.kV = Shooter.SHOOT_kV;
    slot0Configs.kA = Shooter.SHOOT_kA;
    slot0Configs.kP = Shooter.SHOOT_kP;
    slot0Configs.kI = Shooter.SHOOT_kI;
    slot0Configs.kD = Shooter.SHOOT_kD;
    flywheelConfig.Slot0 = slot0Configs;
    flywheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    flywheelLead.getConfigurator().apply(flywheelConfig);
    flywheelLead.setNeutralMode(NeutralModeValue.Coast);

    // Follower config
    flywheelFollower.setControl(
        new Follower(flywheelLead.getDeviceID(), MotorAlignmentValue.Opposed));
    flywheelFollower.setNeutralMode(NeutralModeValue.Coast);

    // Hood config (positional PID or velocity, brake mode, soft stops)
    var hoodConfig = new TalonFXConfiguration();
    var hoodSlot0 = new Slot0Configs();
    hoodSlot0.kP = Shooter.HOOD_kP;
    hoodSlot0.kI = Shooter.HOOD_kI;
    hoodSlot0.kD = Shooter.HOOD_kD;
    hoodSlot0.kV = Shooter.HOOD_kV;
    hoodConfig.Slot0 = hoodSlot0;
    hoodConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Shooter.HOOD_TOP_SOFT_LIMIT_ROT;
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Shooter.HOOD_BOTTOM_SOFT_LIMIT_ROT;
    hoodMotor.getConfigurator().apply(hoodConfig);
    hoodMotor.setNeutralMode(NeutralModeValue.Brake);

    // Turret config (velocity, brake mode, soft stops)
    var turretConfig = new TalonFXConfiguration();
    var turretSlot0 = new Slot0Configs();
    turretSlot0.kP = Shooter.TURRET_kP;
    turretSlot0.kI = Shooter.TURRET_kI;
    turretSlot0.kD = Shooter.TURRET_kD;
    turretSlot0.kV = Shooter.TURRET_kV;
    turretConfig.Slot0 = turretSlot0;
    turretConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    turretConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    turretConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Shooter.TURRET_RIGHT_SOFT_LIMIT_ROT;
    turretConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    turretConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Shooter.TURRET_LEFT_SOFT_LIMIT_ROT;
    turretMotor.getConfigurator().apply(turretConfig);
    turretMotor.setNeutralMode(NeutralModeValue.Brake);

    setupTables();
  }

  @Override
  public void periodic() {
    log();
  }

  // Flywheel velocity (RPS at output)
  public void setFlywheel(double velocityRPS) {
    velocityRPS = velocityRPS * 100; // Temporary scaling for testing
    this.targetFlyWheelVeloRPS = velocityRPS;
    flywheelLead.setControl(leadRequest.withVelocity(velocityRPS));
  }

  /**
   * Check if the flywheel is at the target velocity within tolerance.
   *
   * @return true if flywheel velocity is within tolerance of target
   */
  public boolean isAtVelocity() {
    double currentVelocity = flywheelLead.getVelocity().getValueAsDouble();
    // TAKE A LOOK TO SEE IF SCALING IS NEEDED HERE
    return Math.abs(currentVelocity - targetFlyWheelVeloRPS) < (Shooter.VELOCITY_TOLERANCE_RPS);
  }

  // Hood position (rotations at output)
  public void setHoodPosition(double rotations) {
    this.targetHoodPositionRot = rotations;
    double motorRotations = rotations * Shooter.HOOD_GEAR_RATIO;
    hoodMotor.setControl(hoodPositionRequest.withPosition(motorRotations));
  }

  // Hood velocity (RPS at output)
  public void setHoodVelocity(double rps) {
    double motorRPS = rps * Shooter.HOOD_GEAR_RATIO;
    hoodMotor.setControl(hoodVelocityRequest.withVelocity(motorRPS));
  }

  // Turret position getter
  public double getHoodPosition() {
    return hoodMotor.getPosition().getValueAsDouble() / Shooter.TURRET_GEAR_RATIO;
  }

  public boolean isHoodAtPosition() {
    double currentPosition = getHoodPosition();
    double toleranceRot = Math.toRadians(Shooter.HOOD_TRACKING_TOLERANCE_DEG) / (2 * Math.PI);
    return Math.abs(currentPosition - targetHoodPositionRot) < toleranceRot;
  }

  // Turret position (rotations at output)
  //  public void setTurretPosition(double rotations) {
  //    this.targetTurretPositionRot = rotations;
  //    double motorRotations = rotations * Shooter.TURRET_GEAR_RATIO;
  //    turretMotor.setControl(new PositionVoltage(motorRotations));
  //  }

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
    SmartDashboard.putBoolean("Is Shtr At Spd", isAtVelocity());
    SmartDashboard.putBoolean("Is Trrt At Pos", isTurretAtPosition());
    SmartDashboard.putBoolean("Is Hood At Pos", isHoodAtPosition());

    // Flywheel
    SmartDashboard.putNumber(
        "Shooter Lead Velocity (RPS)", flywheelLead.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber(
        "Shooter Follower Velocity (RPS)", flywheelFollower.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Lead Output", flywheelLead.get());
    SmartDashboard.putNumber("Shooter Follower Output", flywheelFollower.get());
    SmartDashboard.putNumber("Shooter Lead Temp", flywheelLead.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber(
        "Shooter Follower Temp", flywheelFollower.getDeviceTemp().getValueAsDouble());

    // Hood
    SmartDashboard.putNumber(
        "Shooter Hood Position (rot, output)",
        hoodMotor.getPosition().getValueAsDouble() / Shooter.HOOD_GEAR_RATIO);
    SmartDashboard.putNumber(
        "Shooter Hood Velocity (RPS, output)",
        hoodMotor.getVelocity().getValueAsDouble() / Shooter.HOOD_GEAR_RATIO);
    SmartDashboard.putNumber("Hood Temp", hoodMotor.getDeviceTemp().getValueAsDouble());

    // Turret
    SmartDashboard.putNumber(
        "Shooter Turret Position (rot, output)",
        turretMotor.getPosition().getValueAsDouble() / Shooter.TURRET_GEAR_RATIO);
    SmartDashboard.putNumber(
        "Shooter Turret Velocity (RPS, output)",
        turretMotor.getVelocity().getValueAsDouble() / Shooter.TURRET_GEAR_RATIO);
    SmartDashboard.putNumber("Turret Temp", turretMotor.getDeviceTemp().getValueAsDouble());
  }

  private void setupTables() {
    // Alliance Zone Flywheel and Hood Tables // TODO TA: Must empirically obtain values
    // parameters are distance to Hub and RPS velocity of the flywheel
    flyAlliance.put(0.0, 45.0);
    flyAlliance.put(1.0, 45.0);
    flyAlliance.put(2.0, 50.0);
    flyAlliance.put(4.0, 55.0);
    flyAlliance.put(6.0, 60.0);
    flyAlliance.put(8.0, 60.0);
    flyAlliance.put(10.0, 60.0);

    // parameters are distance to Hub and Hood angle
    hoodAlliance.put(0.0, 0.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodAlliance.put(1.0, 0.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodAlliance.put(2.0, 5.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodAlliance.put(4.0, 10.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodAlliance.put(6.0, 15.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodAlliance.put(8.0, 20.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodAlliance.put(10.0, 25.0 / Shooter.HOOD_DEG_PER_ROTATION);

    // Neutral Zone Flywheel and Hood Tables // TODO TA: Must empirically obtain values
    // parameters are distance to Hub and RPS velocity of the flywheel
    flyNeutral.put(0.0, 65.0);
    flyNeutral.put(7.0, 65.0);
    flyNeutral.put(10.0, 65.0);
    flyNeutral.put(15.0, 85.0);

    // parameters are distance to Hub and Hood angle
    hoodNeutral.put(0.0, 45.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodNeutral.put(7.0, 45.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodNeutral.put(10.0, 40.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodNeutral.put(15.0, 40.0 / Shooter.HOOD_DEG_PER_ROTATION);

    // Opposition Zone Flywheel and Hood Tables // TODO TA: Must empirically obtain values
    // parameters are distance to Hub and RPS velocity of the flywheel
    flyOpposition.put(0.0, 85.0);
    flyOpposition.put(12.0, 85.0);
    flyOpposition.put(15.0, 90.0);
    flyOpposition.put(20.0, 90.0);

    // parameters are distance to Hub and Hood angle
    hoodOpposition.put(0.0, 45.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodOpposition.put(10.0, 40.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodOpposition.put(15.0, 45.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodOpposition.put(20.0, 40.0 / Shooter.HOOD_DEG_PER_ROTATION);
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

    flywheelLead.setControl(new VelocityVoltage(flyRPS));
    hoodMotor.setControl(new PositionVoltage(hoodRot * Shooter.HOOD_GEAR_RATIO));
  }

  public void setTurretPosition(double rotations) {
    // SAFETY: Clamp to stop at +/- 135 degrees [Hardware Limit Check]
    double clampedRot =
        MathUtil.clamp(rotations, -Shooter.TURRET_LIMIT_ROT, Shooter.TURRET_LIMIT_ROT);
    turretMotor.setControl(new PositionVoltage(clampedRot * Shooter.TURRET_GEAR_RATIO));
  }

  public void setTurretWithFF(double rotations, double feedforwardRPS) {
    double clamped = MathUtil.clamp(rotations, -Shooter.TURRET_LIMIT_ROT, Shooter.TURRET_LIMIT_ROT);
    this.targetTurretRot = clamped;
    double ffVolts = feedforwardRPS * Shooter.TURRET_GEAR_RATIO * Shooter.TURRET_kV;

    turretMotor.setControl(
        new PositionVoltage(clamped * Shooter.TURRET_GEAR_RATIO).withFeedForward(ffVolts));
  }

  public double getVisionCorrectedRotation(double odoRot) {
    var result = camera.getLatestResult();
    if (result.hasTargets()) {
      return (turretMotor.getPosition().getValueAsDouble() / Shooter.TURRET_GEAR_RATIO)
          + (result.getBestTarget().getYaw() / 360.0);
    }
    return odoRot;
  }
}
