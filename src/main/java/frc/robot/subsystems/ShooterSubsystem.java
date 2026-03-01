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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
  // Shuffleboard entries
  private final ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
  private final ShuffleboardTab canTab = Shuffleboard.getTab("CAN Status");
  private final GenericEntry shtrAtSpdEntry;
  private final GenericEntry trrtAtPosEntry;
  private final GenericEntry hoodAtPosEntry;
  private final GenericEntry shtrLeadVeloEntry;
  private final GenericEntry shtrFlwrVeloEntry;
  private final GenericEntry shtrLeadCmdEntry;
  private final GenericEntry shtrFlwrCmdEntry;
  private final GenericEntry shtrLeadTempEntry;
  private final GenericEntry shtrFlwrTempEntry;
  private final GenericEntry hoodPosEntry;
  private final GenericEntry hoodVeloEntry;
  private final GenericEntry hoodTempEntry;
  private final GenericEntry turretPosEntry;
  private final GenericEntry turretVeloEntry;
  private final GenericEntry turretTempEntry;
  private final GenericEntry shtrCanOkEntry;
  private final GenericEntry flywhCanOkEntry;
  private final GenericEntry trrtCanOkEntry;
  private final GenericEntry hoodCanOkEntry;
  private final GenericEntry readyToShoot;
  private final GenericEntry distanceEntry;

  public ShooterSubsystem() {

    distanceEntry =
        shooterTab
            .add("Distance To Target", 0.0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(6, 6)
            .withSize(2, 1)
            .getEntry();

    configureMotors();
    setupTables();

    turretPositionSignal = turretMotor.getPosition();
    flywheelVelocitySignal = flywheelLead.getVelocity();
    hoodPositionSignal = hoodMotor.getPosition();

    // Create persistent Shuffleboard widgets
    shtrAtSpdEntry =
        shooterTab
            .add("Shtr Is At Spd", isFLywheelAtVelocity())
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(0, 0)
            .withSize(2, 1)
            .getEntry();
    trrtAtPosEntry =
        shooterTab
            .add("Trrt Is At Pos", isTurretAtPosition())
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(2, 0)
            .withSize(2, 1)
            .getEntry();
    hoodAtPosEntry =
        shooterTab
            .add("Hood Is At Pos", isHoodAtPosition())
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(4, 0)
            .withSize(2, 1)
            .getEntry();

    readyToShoot =
        shooterTab
            .add("Ready to Shoot", isReadyToShoot())
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(6, 0)
            .withSize(2, 1)
            .getEntry();

    shtrLeadVeloEntry =
        shooterTab
            .add("Shtr Lead Velo (RPS)", flywheelLead.getVelocity().getValueAsDouble())
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 1)
            .withSize(2, 1)
            .getEntry();
    shtrFlwrVeloEntry =
        shooterTab
            .add("Shtr Flwr Velo (RPS)", flywheelFollower.getVelocity().getValueAsDouble())
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(2, 1)
            .withSize(2, 1)
            .getEntry();
    shtrLeadCmdEntry =
        shooterTab
            .add("Shtr Lead Cmd", flywheelLead.get())
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(4, 1)
            .withSize(2, 1)
            .getEntry();
    shtrFlwrCmdEntry =
        shooterTab
            .add("Shtr Flwr Cmd", flywheelFollower.get())
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(6, 1)
            .withSize(2, 1)
            .getEntry();
    shtrLeadTempEntry =
        shooterTab
            .add("Shtr Lead Temp", flywheelLead.getDeviceTemp().getValueAsDouble())
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 2)
            .withSize(2, 1)
            .getEntry();
    shtrFlwrTempEntry =
        shooterTab
            .add("Shtr Flwr Temp", flywheelFollower.getDeviceTemp().getValueAsDouble())
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(2, 2)
            .withSize(2, 1)
            .getEntry();

    hoodPosEntry =
        shooterTab
            .add(
                "Shtr Hood Pos (deg, output)",
                hoodMotor.getPosition().getValueAsDouble()) // * 360.0 / Shooter.HOOD_GEAR_RATIO)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 3)
            .withSize(2, 1)
            .getEntry();
    hoodVeloEntry =
        shooterTab
            .add("Shtr Hood Velo (RPS, output)", hoodMotor.getVelocity().getValueAsDouble())
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(2, 3)
            .withSize(2, 1)
            .getEntry();
    hoodTempEntry =
        shooterTab
            .add("Hood Temp", hoodMotor.getDeviceTemp().getValueAsDouble())
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(4, 3)
            .withSize(2, 1)
            .getEntry();

    turretPosEntry =
        shooterTab
            .add(
                "Shtr Turret Pos (deg, output)",
                turretMotor.getPosition().getValueAsDouble() * 360.0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 4)
            .withSize(2, 1)
            .getEntry();
    turretVeloEntry =
        shooterTab
            .add("Shtr Turret Velo (RPS, output)", turretMotor.getVelocity().getValueAsDouble())
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(2, 4)
            .withSize(2, 1)
            .getEntry();
    turretTempEntry =
        shooterTab
            .add("Turret Temp", turretMotor.getDeviceTemp().getValueAsDouble())
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(4, 4)
            .withSize(2, 1)
            .getEntry();

    shtrCanOkEntry =
        canTab
            .add("Shtr CAN OK", flywheelVelocitySignal.getStatus().isOK())
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(0, 4)
            .withSize(2, 1)
            .getEntry();
    flywhCanOkEntry =
        canTab
            .add("FlyWh CAN OK", flywheelVelocitySignal.getStatus().isOK())
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(2, 4)
            .withSize(1, 1)
            .getEntry();
    trrtCanOkEntry =
        canTab
            .add("Trrt CAN OK", turretPositionSignal.getStatus().isOK())
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(3, 4)
            .withSize(1, 1)
            .getEntry();
    hoodCanOkEntry =
        canTab
            .add("Hood CAN OK", hoodPositionSignal.getStatus().isOK())
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(4, 4)
            .withSize(1, 1)
            .getEntry();
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
    leadConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
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

    // 1. RATIO: 11:1 on Belt drive
    turretConfigs.Feedback.SensorToMechanismRatio = 11.0;

    // 2. FRICTION (kS) - THE MOST IMPORTANT TUNING PARAMETER
    // If you have "huge friction", 0.25V is likely too low.
    // Increase this until the turret *just barely* moves when you enable, then back off 0.05.
    // For a sticky belt, this might need to be 0.45 - 0.60 Volts.
    turretConfigs.Slot0.kS = 0.25;

    // 3. VELOCITY FF (kV)
    // 0.12 Volts / (1 Rot/s) -> At max speed (9 RPS), this adds ~1.08V.
    // This seems low. Ideally, 12V should correspond to Max Speed.
    // Try: 12V / 9 RPS = ~1.33.
    // Start conservative:
    turretConfigs.Slot0.kV = 1.32;

    // 4. PROPORTIONAL (kP)
    // 200 is very "stiff" (200V per 1 rotation error).
    // With high friction, stiff is good, but 200 might cause the vibration.
    // If it buzzes, drop this to 80-120.
    turretConfigs.Slot0.kP = 120.0;
    turretConfigs.Slot0.kI = 0.0; // Keep zero!
    turretConfigs.Slot0.kD = 0.0; // Add small D (e.g., 5.0) only if it oscillates.

    // 5. MOTION MAGIC - UNLEASH THE SPEED
    // Cruise: 4.0 RPS (1440 deg/s) - Must be faster than Robot Spin (720 deg/s)
    turretConfigs.MotionMagic.MotionMagicCruiseVelocity = 4.0;

    // Accel: 10.0 RPS/s - Snap to target in < 0.2 seconds
    turretConfigs.MotionMagic.MotionMagicAcceleration = 10.0;

    // Jerk: 100.0 or 0. Smooths the start/stop to prevent belt slip.
    // 0 = Infinite Jerk (Max Aggression). For vision, higher jerk is often better.
    turretConfigs.MotionMagic.MotionMagicJerk = 0.0;

    // 6. DEADBAND
    turretConfigs.MotorOutput.DutyCycleNeutralDeadband = 0.001;

    // ... Limits & Inverts ...
    turretConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    turretConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Shooter.TURRET_RIGHT_SOFT_LIMIT_ROT;
    turretConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    turretConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Shooter.TURRET_LEFT_SOFT_LIMIT_ROT;
    turretConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    turretMotor.getConfigurator().apply(turretConfigs);

    turretMotor.setNeutralMode(NeutralModeValue.Brake);

    // --- Hood (Motion Magic) ---
    TalonFXConfiguration hoodConfigs = new TalonFXConfiguration();

    hoodConfigs.Feedback.SensorToMechanismRatio = 45.0; // Internal Reduction

    // hoodConfigs.Slot0.kP = 1.5;
    // hoodConfigs.Slot0.kV = 0.12;
    // hoodConfigs.MotionMagic.MotionMagicCruiseVelocity = 40;
    // hoodConfigs.MotionMagic.MotionMagicAcceleration = 80;

    hoodConfigs.Slot0.kS = 0.7; // Voltage to overcome friction
    hoodConfigs.Slot0.kV = .12; // Theoretical kV for Kraken
    hoodConfigs.Slot0.kP = 200; // Higher P for precision positioning
    hoodConfigs.MotionMagic.MotionMagicCruiseVelocity = 8; //  RPS
    hoodConfigs.MotionMagic.MotionMagicAcceleration = 040;

    hoodConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
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
  public void setFlywheel(double velocity) {
    double velocityRPS = velocity * 100; // Temporary scaling for testing
    this.targetFlyWheelVeloRPS = velocityRPS;
    flywheelLead.setControl(leadRequest.withVelocity(velocityRPS));
  }

  public void setFlywheelRPS(double velocityRPS) {
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

    distanceEntry.setValue(distance);
  }

  public void setFlywheelViaTable(double distance, String zone) {
    double flyRPS =
        zone.equals("Alliance")
            ? flyAlliance.get(distance)
            : zone.equals("Neutral") ? flyNeutral.get(distance) : flyOpposition.get(distance);
    flywheelLead.setControl(m_velocityReq.withVelocity(flyRPS));

    distanceEntry.setValue(distance);
  }

  public void setHoodViaTable(double distance, String zone) {
    double hoodRot =
        zone.equals("Alliance")
            ? hoodAlliance.get(distance)
            : zone.equals("Neutral") ? hoodNeutral.get(distance) : hoodOpposition.get(distance);
    hoodMotor.setControl(m_mmReq.withPosition(hoodRot));

    distanceEntry.setValue(distance);
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
    return hoodMotor.getPosition().getValueAsDouble();
  }

  public boolean isHoodAtPosition() {
    double currentPosition = getHoodPosition();
    return Math.abs(currentPosition - targetHoodPositionRot) < Shooter.HOOD_TRACKING_TOLERANCE_ROT;
  }

  public void setTurretPosition(double rotations) {
    // SAFETY: Clamp to stop at +/- 135 degrees [Hardware Limit Check]
    double clampedRot =
        //        MathUtil.clamp(rotations, -Shooter.TURRET_LIMIT_ROT, Shooter.TURRET_LIMIT_ROT);
        tomClamp(rotations);
    turretMotor.setControl(m_mmReq.withPosition(clampedRot));
  }
  // Currently not used
  public void setTurretWithFF(double rotations, double feedforwardRPS) {
    double clamped =
        //    MathUtil.clamp(rotations, -Shooter.TURRET_LIMIT_ROT, Shooter.TURRET_LIMIT_ROT);
        tomClamp(rotations);
    this.targetTurretRot = clamped;
    double ffVolts = feedforwardRPS * Shooter.TURRET_kV;

    // TODO: TA - Decide on Magic Motion or Straight PID for Turret angle
    // turretMotor.setControl(
    //     new PositionVoltage(clamped).withFeedForward(ffVolts));
    turretMotor.setControl(m_mmReq.withPosition(clamped));
  }

  public double tomClamp(double rotations) {
    double clampedRot;
    if (rotations > 1) rotations = rotations % 1;
    if (rotations < -1) rotations = rotations % 1;
    this.targetTurretPositionRot = rotations;
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
    double motorRPS = rps;
    turretMotor.setControl(turretVelocityRequest.withVelocity(motorRPS));
  }

  // Turret position getter
  public double getTurretPosition() {
    return turretMotor.getPosition().getValueAsDouble();
  }

  public boolean isTurretAtPosition() {
    double currentPosition = getTurretPosition();
    double toleranceRot = Math.toRadians(Shooter.TURRET_TRACKING_TOLERANCE_DEG) / (2 * Math.PI);
    return Math.abs(currentPosition - targetTurretPositionRot) < toleranceRot;
  }

  public boolean isReadyToShoot() {
    if (isTurretAtPosition() && isFLywheelAtVelocity() && isHoodAtPosition()) {
      return true;
    } else {
      return false;
    }
  }

  public Rotation2d getAngleToHub(Pose2d robotPose) {
    var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    Translation2d target =
        (alliance == DriverStation.Alliance.Red) ? Shooter.RED_HUB_POS : Shooter.BLUE_HUB_POS;

    Translation2d relTrans = target.minus(robotPose.getTranslation());
    return new Rotation2d(Math.atan2(relTrans.getY(), relTrans.getX()));
  }

  public void stop() {
    flywheelLead.stopMotor();
    flywheelFollower.stopMotor();
    hoodMotor.stopMotor();
    turretMotor.stopMotor();
  }

  public void log() {
    // Update persistent entries
    shtrAtSpdEntry.setBoolean(isFLywheelAtVelocity());
    trrtAtPosEntry.setBoolean(isTurretAtPosition());
    hoodAtPosEntry.setBoolean(isHoodAtPosition());

    shtrLeadVeloEntry.setDouble(flywheelLead.getVelocity().getValueAsDouble());
    shtrFlwrVeloEntry.setDouble(flywheelFollower.getVelocity().getValueAsDouble());
    shtrLeadCmdEntry.setDouble(flywheelLead.get());
    shtrFlwrCmdEntry.setDouble(flywheelFollower.get());
    shtrLeadTempEntry.setDouble(flywheelLead.getDeviceTemp().getValueAsDouble());
    shtrFlwrTempEntry.setDouble(flywheelFollower.getDeviceTemp().getValueAsDouble());

    hoodPosEntry.setDouble(
        hoodMotor.getPosition().getValueAsDouble()); // TA * 360.0 / Shooter.HOOD_GEAR_RATIO);
    hoodVeloEntry.setDouble(hoodMotor.getVelocity().getValueAsDouble());
    hoodTempEntry.setDouble(hoodMotor.getDeviceTemp().getValueAsDouble());

    turretPosEntry.setDouble(turretMotor.getPosition().getValueAsDouble() * 360.0);
    turretVeloEntry.setDouble(turretMotor.getVelocity().getValueAsDouble());
    turretTempEntry.setDouble(turretMotor.getDeviceTemp().getValueAsDouble());

    // ── CAN Status page ───────────────────────────────────────────────────────
    boolean shooterOK = flywheelVelocitySignal.getStatus().isOK();
    boolean turretOK = turretPositionSignal.getStatus().isOK();
    boolean hoodOK = hoodPositionSignal.getStatus().isOK();

    // Update CAN status entries
    shtrCanOkEntry.setBoolean(shooterOK && turretOK && hoodOK);
    flywhCanOkEntry.setBoolean(shooterOK);
    trrtCanOkEntry.setBoolean(turretOK);
    hoodCanOkEntry.setBoolean(hoodOK);
  }

  private void setupTables() {
    // Alliance Zone Flywheel and Hood Tables // TODO TA: Must empirically obtain values
    // parameters are distance to Hub in METERS and RPS velocity of the flywheel
    flyAlliance.put(0.0, 40.0);
    flyAlliance.put(1.0, 40.0); // ABOUT THE SHORTEST SHOT IN OUR ALLIANCE ZONE
    flyAlliance.put(2.0, 45.0);
    flyAlliance.put(4.0, 50.0);
    flyAlliance.put(6.0, 55.0); // ABOUT THE LONGEST SHOT IN OUR ALLIANCE ZONE
    flyAlliance.put(8.0, 60.0);

    // parameters are distance to Hub in METERS and Hood angle in rotations
    hoodAlliance.put(0.0, 0.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodAlliance.put(1.0, 0.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodAlliance.put(2.0, 5.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodAlliance.put(4.0, 10.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodAlliance.put(6.0, 15.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodAlliance.put(8.0, 20.0 / Shooter.HOOD_DEG_PER_ROTATION);

    // Neutral Zone Flywheel and Hood Tables // TODO TA: Must empirically obtain values
    // parameters are distance to Hub in METERS and RPS velocity of the flywheel
    flyNeutral.put(0.0, 55.0);
    flyNeutral.put(4.0, 55.0); // ABOUT THE SHORTEST SHOT IN THE NEUTRAL ZONE
    flyNeutral.put(7.0, 60.0);
    flyNeutral.put(10.0, 65.0);
    flyNeutral.put(12.0, 70.0); // ABOUT THE LONGEST SHOT IN THE NEUTRAL ZONE
    flyNeutral.put(15.0, 75.0);

    // parameters are distance to Hub in METERS and Hood angle in rotation
    hoodNeutral.put(0.0, 10.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodNeutral.put(4.0, 10.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodNeutral.put(7.0, 12.5 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodNeutral.put(10.0, 15.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodNeutral.put(12.0, 17.5 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodNeutral.put(15.0, 20.0 / Shooter.HOOD_DEG_PER_ROTATION);

    // Opposition Zone Flywheel and Hood Tables // TODO TA: Must empirically obtain values
    // parameters are distance to Hub in METERS and RPS velocity of the flywheel
    flyOpposition.put(0.0, 65.0);
    flyOpposition.put(10.0, 65.0); // ABOUT THE SHORTEST SHOT IN OPPONENTS ZONE
    flyOpposition.put(12.0, 70.0);
    flyOpposition.put(15.0, 75.0);
    flyOpposition.put(20.0, 80.0);

    // parameters are distance to Hub in METERS and Hood angle in rotation
    hoodOpposition.put(0.0, 15.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodOpposition.put(10.0, 15.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodOpposition.put(12.0, 17.5 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodOpposition.put(15.0, 20.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodOpposition.put(20.0, 20.0 / Shooter.HOOD_DEG_PER_ROTATION);
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
