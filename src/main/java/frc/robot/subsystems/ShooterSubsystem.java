package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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

  private final VelocityVoltage leadRequest = new VelocityVoltage(0);
  private final PositionVoltage hoodPositionRequest = new PositionVoltage(0);
  private final VelocityVoltage hoodVelocityRequest = new VelocityVoltage(0);

  // --- Control Requests ---
  private final VelocityVoltage m_velocityReq = new VelocityVoltage(0);
  private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);

  private double targetFlyWheelVeloRPS = 0.0;
  private double targetHoodPositionRot = 0.0;

  // 6 Interpolation Maps
  private final InterpolatingDoubleTreeMap flyAlliance = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap hoodAlliance = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap flyNeutral = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap hoodNeutral = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap flyOpposition = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap hoodOpposition = new InterpolatingDoubleTreeMap();

  private final com.ctre.phoenix6.StatusSignal<AngularVelocity> flywheelVelocitySignal;
  private final com.ctre.phoenix6.StatusSignal<Angle> hoodPositionSignal;
  // Shuffleboard entries
  private final ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
  private final ShuffleboardTab canTab = Shuffleboard.getTab("CAN Status");
  private final GenericEntry shtrAtSpdEntry;
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
  private final GenericEntry shtrCanOkEntry;
  private final GenericEntry flywhCanOkEntry;
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

    flywheelVelocitySignal = flywheelLead.getVelocity();
    hoodPositionSignal = hoodMotor.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(50, flywheelVelocitySignal, hoodPositionSignal);

    // Create persistent Shuffleboard widgets
    shtrAtSpdEntry =
        shooterTab
            .add("Shtr Is At Spd", isFLywheelAtVelocity())
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(0, 0)
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
    leadConfigs.withCurrentLimits(
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(70))
            .withStatorCurrentLimitEnable(true));
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

    followerConfigs.withCurrentLimits(
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(70))
            .withStatorCurrentLimitEnable(true));
    flywheelFollower.getConfigurator().apply(followerConfigs);

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

    hoodConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    hoodConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Keep Brake for hood
    hoodConfigs.MotorOutput.DutyCycleNeutralDeadband = 0.01; // Help reach the final point

    hoodConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    hoodConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Shooter.HOOD_TOP_SOFT_LIMIT_ROT;
    hoodConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    hoodConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Shooter.HOOD_BOTTOM_SOFT_LIMIT_ROT;

    hoodConfigs.withCurrentLimits(
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(60))
            .withStatorCurrentLimitEnable(true));
    hoodMotor.getConfigurator().apply(hoodConfigs);
    hoodMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setCoastMode() {
    hoodMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  public void setBrakeMode() {
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

  public boolean isReadyToShoot() {
    if (isFLywheelAtVelocity() && isHoodAtPosition()) {
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
  }

  public void log() {
    // Update persistent entries
    shtrAtSpdEntry.setBoolean(isFLywheelAtVelocity());
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

    // ── CAN Status page ───────────────────────────────────────────────────────
    boolean shooterOK = flywheelVelocitySignal.getStatus().isOK();
    boolean hoodOK = hoodPositionSignal.getStatus().isOK();

    // Update CAN status entries
    shtrCanOkEntry.setBoolean(shooterOK && hoodOK);
    flywhCanOkEntry.setBoolean(shooterOK);
    hoodCanOkEntry.setBoolean(hoodOK);
  }

  private void setupTables() {
    // Alliance Zone Flywheel and Hood Tables // TODO TA: Must empirically obtain values
    // parameters are distance to Hub in METERS and RPS velocity of the flywheel
    flyAlliance.put(0.0, 40.0);
    flyAlliance.put(1.0, 40.0); // ABOUT THE SHORTEST SHOT IN OUR ALLIANCE ZONE
    flyAlliance.put(2.0, 45.0);
    flyAlliance.put(3.86, 50.0);
    flyAlliance.put(5.62, 57.0); // outpost
    // flyAlliance.put(6.0, 60.0); // ABOUT THE LONGEST SHOT IN OUR ALLIANCE ZONE
    flyAlliance.put(8.0, 70.0);

    // parameters are distance to Hub in METERS and Hood angle in rotations
    hoodAlliance.put(0.0, 0.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodAlliance.put(1.0, 0.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodAlliance.put(2.56, 0.35 / 4.0);
    hoodAlliance.put(4.0, 8.0 / 4.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodAlliance.put(6.0, 12.0 / 4.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodAlliance.put(8.0, 14.0 / 4.0 / Shooter.HOOD_DEG_PER_ROTATION);

    // Neutral Zone Flywheel and Hood Tables // TODO TA: Must empirically obtain values
    // parameters are distance to Hub in METERS and RPS velocity of the flywheel
    flyNeutral.put(0.0, 50.0);
    flyNeutral.put(4.0, 52.0); // ABOUT THE SHORTEST SHOT IN THE NEUTRAL ZONE
    flyNeutral.put(7.0, 65.0);
    flyNeutral.put(10.0, 75.0);
    flyNeutral.put(12.0, 80.0); // ABOUT THE LONGEST SHOT IN THE NEUTRAL ZONE
    flyNeutral.put(15.0, 85.0);

    // parameters are distance to Hub in METERS and Hood angle in rotation
    hoodNeutral.put(0.0, 8.0 / 2.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodNeutral.put(4.0, 8.0 / 2.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodNeutral.put(7.0, 10.5 / 2.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodNeutral.put(10.0, 13.0 / 2.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodNeutral.put(12.0, 13.5 / 2.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodNeutral.put(15.0, 14.0 / 2.0 / Shooter.HOOD_DEG_PER_ROTATION);

    // Opposition Zone Flywheel and Hood Tables // TODO TA: Must empirically obtain values
    // parameters are distance to Hub in METERS and RPS velocity of the flywheel
    flyOpposition.put(0.0, 65.0);
    flyOpposition.put(10.0, 75.0); // ABOUT THE SHORTEST SHOT IN OPPONENTS ZONE
    flyOpposition.put(12.0, 80.0);
    flyOpposition.put(15.0, 85.0); // ABOUT THE LONGEST SHOT IN OPPONENTS ZONE
    flyOpposition.put(20.0, 90.0);

    // parameters are distance to Hub in METERS and Hood angle in rotation
    hoodOpposition.put(0.0, 13.0 / 2.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodOpposition.put(10.0, 13.0 / 2.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodOpposition.put(12.0, 13.5 / 2.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodOpposition.put(15.0, 14.0 / 2.0 / Shooter.HOOD_DEG_PER_ROTATION);
    hoodOpposition.put(20.0, 14.5 / 2.0 / Shooter.HOOD_DEG_PER_ROTATION);
  }

  public List<BaseStatusSignal> getSignals() {
    // Explicitly define the list type to avoid generic mismatch
    return List.of(
        (BaseStatusSignal) flywheelVelocitySignal, (BaseStatusSignal) hoodPositionSignal);
  }

  public boolean isShooterConnected() {
    // Uses the checks we already built using getStatus().isOK()
    return flywheelVelocitySignal.getStatus().isOK() && hoodPositionSignal.getStatus().isOK();
  }
}
