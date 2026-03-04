package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.Harvester.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Harvester;
import java.util.Map;

public class HarvesterSubsystem extends SubsystemBase {
  private final TalonFX deployMotor = new TalonFX(Harvester.DEPLOY_MOTOR_ID);
  private final TalonFX spinMotor = new TalonFX(Harvester.SPIN_MOTOR_ID);

  private final PositionVoltage deployPositionRequest = new PositionVoltage(0);
  private final VelocityVoltage deployVelocityRequest = new VelocityVoltage(0);
  private final VelocityVoltage spinVelocityRequest = new VelocityVoltage(0);

  private final com.ctre.phoenix6.StatusSignal<AngularVelocity> spinVelocitySignal;
  private final com.ctre.phoenix6.StatusSignal<Angle> deployPositionSignal;

  // Shuffleboard entries
  private final ShuffleboardTab harvTab = Shuffleboard.getTab("Harvester");
  private final GenericEntry harvDeployPosEntry;
  private final GenericEntry harvDeployVeloEntry;
  private final GenericEntry harvDeployTempEntry;
  private final GenericEntry harvSpinVeloEntry;
  private final GenericEntry harvSpinTempEntry;
  private final GenericEntry harvDeployCanOkEntry;
  private final GenericEntry harvSpinCanOkEntry;
  private final GenericEntry harvDeployCurrentEntry;
  private final GenericEntry harvDeployEndstopEntry;
  // Live tuners (safe, bounded) for homing / tuning
  private final GenericEntry harvDeployHomingCurrentEntry;
  private final GenericEntry harvDeployHomingConsecEntry;
  private final GenericEntry harvDeployHomingTimeoutEntry;
  private final GenericEntry harvDeployCruiseRPSEntry;

  private final MotionMagicVoltage m_mmReqLt = new MotionMagicVoltage(0);

  public HarvesterSubsystem() {

    // Deploy motor config (position/velocity, brake)
    var deployConfig = new com.ctre.phoenix6.configs.TalonFXConfiguration();
    var deploySlot0 = new Slot0Configs();
    // deploySlot0.kP = deploykP;
    // deploySlot0.kI = deploykI;
    // deploySlot0.kD = deploykD;
    // deploySlot0.kV = deploykV;
    deploySlot0.kS = 0.25; // Add 0.25 V output to overcome static friction
    deploySlot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    deploySlot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    deploySlot0.kP = 60; // A position error of 0.2 rotations results in 12 V output
    deploySlot0.kI = 0; // No output for integrated error
    deploySlot0.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output

    deployConfig.Slot0 = deploySlot0;
    deployConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    deployMotor.getConfigurator().apply(deployConfig);
    deployMotor.setNeutralMode(NeutralModeValue.Brake);

    /* Configure gear ratio */
    FeedbackConfigs fdb = deployConfig.Feedback;
    fdb.SensorToMechanismRatio = 20.0; // 20 rotor rotations per mechanism rotation
    setDeployPosition(DEPLOY_START_ANGLE); // Start at Zero position

    /* Configure Motion Magic */
    MotionMagicConfigs mm = deployConfig.MotionMagic;
    mm.withMotionMagicCruiseVelocity(
            RotationsPerSecond.of(DEPLOY_MM_CRUISE_RPS)) // mechanism rotations per second cruise
        .withMotionMagicAcceleration(
            RotationsPerSecondPerSecond.of(DEPLOY_MM_ACCEL_RPSPS)) // mechanism rotations/sec^2
        // Motion magic jerk (not critical for now)
        .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(DEPLOY_MM_JERK));

    deployConfig.withCurrentLimits(
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(80))
            .withStatorCurrentLimitEnable(true));

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = deployMotor.getConfigurator().apply(deployConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure Harv Deploy device. Error: " + status.toString());
    }

    deployMotor.setNeutralMode(NeutralModeValue.Brake);

    // Spin motor config (velocity, coast)
    var spinConfig = new com.ctre.phoenix6.configs.TalonFXConfiguration();
    var spinSlot0 = new Slot0Configs();
    spinSlot0.kP = spinkP;
    spinSlot0.kI = spinkI;
    spinSlot0.kD = spinkD;
    spinSlot0.kV = spinkV;
    spinConfig.Slot0 = spinSlot0;
    spinConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    spinMotor.getConfigurator().apply(spinConfig);
    spinMotor.setNeutralMode(NeutralModeValue.Coast);

    spinVelocitySignal = spinMotor.getVelocity();
    deployPositionSignal = deployMotor.getPosition();
    // Persistent Shuffleboard widgets
    harvDeployPosEntry =
        harvTab
            .add(
                "Harv Deploy Pos (deg)",
                deployMotor.getPosition().getValueAsDouble() / DEPLOY_DEGREES_TO_ROTATIONS)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 0)
            .withSize(2, 1)
            .getEntry();
    harvDeployVeloEntry =
        harvTab
            .add(
                "Harv Deploy Velo (RPS, output)",
                deployMotor.getVelocity().getValueAsDouble() / Harvester.DEPLOY_GEAR_RATIO)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(2, 0)
            .withSize(2, 1)
            .getEntry();
    harvDeployTempEntry =
        harvTab
            .add("Harv Deploy Temp", deployMotor.getDeviceTemp().getValueAsDouble())
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(4, 0)
            .withSize(2, 1)
            .getEntry();

    harvSpinVeloEntry =
        harvTab
            .add(
                "Harv Spin Velo (RPS, output)",
                spinMotor.getVelocity().getValueAsDouble() / Harvester.SPIN_GEAR_RATIO)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 1)
            .withSize(2, 1)
            .getEntry();
    harvSpinTempEntry =
        harvTab
            .add("Harv Spin Temp", spinMotor.getDeviceTemp().getValueAsDouble())
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(2, 1)
            .withSize(2, 1)
            .getEntry();

    harvDeployCurrentEntry =
        harvTab
            .add("Harv Deploy Current (A)", deployMotor.getStatorCurrent().getValueAsDouble())
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(4, 1)
            .withSize(2, 1)
            .getEntry();

    harvDeployEndstopEntry =
        harvTab
            .add("Harv Deploy Endstop Hit", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(6, 1)
            .withSize(1, 1)
            .getEntry();

    // Tuners (number sliders / boxes) - use safe bounds via properties
    harvDeployHomingCurrentEntry =
        harvTab
            .add("Deploy Homing Current (A)", Harvester.DEPLOY_HOMING_CURRENT_AMPS)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withPosition(0, 2)
            .withSize(2, 1)
            .withProperties(Map.of("min", 5, "max", 80))
            .getEntry();

    harvDeployHomingConsecEntry =
        harvTab
            .add("Deploy Homing Consecutive Cycles", Harvester.DEPLOY_HOMING_CONSECUTIVE_CYCLES)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withPosition(2, 2)
            .withSize(2, 1)
            .withProperties(Map.of("min", 1, "max", 20))
            .getEntry();

    harvDeployHomingTimeoutEntry =
        harvTab
            .add("Deploy Homing Timeout (s)", Harvester.DEPLOY_HOMING_TIMEOUT_S)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withPosition(4, 2)
            .withSize(2, 1)
            .withProperties(Map.of("min", 0, "max", 10))
            .getEntry();

    harvDeployCruiseRPSEntry =
        harvTab
            .add("Deploy Cruise RPS", Harvester.DEPLOY_MM_CRUISE_RPS)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withPosition(6, 2)
            .withSize(2, 1)
            .withProperties(Map.of("min", 0, "max", 6))
            .getEntry();

    var canTab = Shuffleboard.getTab("CAN Status");
    harvDeployCanOkEntry =
        canTab
            .add("Deply CAN OK", deployPositionSignal.getStatus().isOK())
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(0, 5)
            .withSize(1, 1)
            .getEntry();
    harvSpinCanOkEntry =
        canTab
            .add("Spin CAN OK", spinVelocitySignal.getStatus().isOK())
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(1, 5)
            .withSize(1, 1)
            .getEntry();
  }

  @Override
  public void periodic() {
    log();
  }

  /* Set Deploy motor Position using Magic Motion*/
  public void setHarvDeployMagicMoPos(double pos) {
    deployMotor.setControl(
        m_mmReqLt.withPosition(pos).withSlot(0).withOverrideBrakeDurNeutral(true));
  }

  // Deploy position control (shaft rotations at output)
  // Note that motor rotations to shaft rotations ratio is handled in motor config
  public void setDeployPosition(double degrees) {
    double shaftRotations = degrees * Harvester.DEPLOY_DEGREES_TO_ROTATIONS;
    deployMotor.setControl(deployPositionRequest.withPosition(shaftRotations));
  }

  // Deploy velocity control (RPS at output)
  public void setDeployVelocity(double rps) {
    double motorRPS = rps * Harvester.DEPLOY_GEAR_RATIO;
    deployMotor.setControl(deployVelocityRequest.withVelocity(motorRPS));
  }

  // Spin velocity control (RPS at output)
  public void setSpinVelocity(double rps) {
    double motorRPS = rps * Harvester.SPIN_GEAR_RATIO;
    spinMotor.setControl(spinVelocityRequest.withVelocity(motorRPS));
  }

  public void stopDeploy() {
    deployMotor.stopMotor();
  }

  public void stopSpin() {
    spinMotor.stopMotor();
  }

  public void stopHarvester() {
    spinMotor.stopMotor();
    deployMotor.stopMotor();
  }

  public void log() {
    // Update persistent entries
    harvDeployPosEntry.setDouble(
        deployMotor.getPosition().getValueAsDouble() / DEPLOY_DEGREES_TO_ROTATIONS);
    harvDeployVeloEntry.setDouble(
        deployMotor.getVelocity().getValueAsDouble() / Harvester.DEPLOY_GEAR_RATIO);
    harvDeployTempEntry.setDouble(deployMotor.getDeviceTemp().getValueAsDouble());

    harvDeployCurrentEntry.setDouble(deployMotor.getStatorCurrent().getValueAsDouble());
    harvDeployEndstopEntry.setBoolean(deployEndstopHit);

    harvSpinVeloEntry.setDouble(
        spinMotor.getVelocity().getValueAsDouble() / Harvester.SPIN_GEAR_RATIO);
    harvSpinTempEntry.setDouble(spinMotor.getDeviceTemp().getValueAsDouble());

    boolean deployOK = deployPositionSignal.getStatus().isOK();
    boolean spinOK = spinVelocitySignal.getStatus().isOK();
    harvDeployCanOkEntry.setBoolean(deployOK);
    harvSpinCanOkEntry.setBoolean(spinOK);
  }

  private volatile boolean deployEndstopHit = false;

  /**
   * Set the 'deploy endstop hit' flag (for Shuffleboard/telemetry). Commands should set this to
   * true when they detect a stall/hit; clearing is handled on initialize of commands that perform
   * homing.
   */
  public void setDeployEndstopHit(boolean hit) {
    deployEndstopHit = hit;
  }

  public boolean getDeployEndstopHit() {
    return deployEndstopHit;
  }

  public boolean isHarvesterConnected() {
    // Uses the checks we already built using getStatus().isOK()
    return spinVelocitySignal.getStatus().isOK() && deployPositionSignal.getStatus().isOK();
  }

  // Tunable getters: read the Shuffleboard entries (fall back to Constants) and clamp to safe
  // ranges.
  public double getDeployHomingCurrentAmps() {
    double val = harvDeployHomingCurrentEntry.getDouble(Harvester.DEPLOY_HOMING_CURRENT_AMPS);
    // Clamp between 5 A and the stator limit (80 A)
    return Math.max(5.0, Math.min(val, 80.0));
  }

  public int getDeployHomingConsecutiveCycles() {
    double val = harvDeployHomingConsecEntry.getDouble(Harvester.DEPLOY_HOMING_CONSECUTIVE_CYCLES);
    int v = (int) Math.round(val);
    return Math.max(1, Math.min(v, 20));
  }

  public double getDeployHomingTimeoutS() {
    double val = harvDeployHomingTimeoutEntry.getDouble(Harvester.DEPLOY_HOMING_TIMEOUT_S);
    return Math.max(0.0, Math.min(val, 10.0));
  }

  public double getDeployCruiseRPS() {
    double val = harvDeployCruiseRPSEntry.getDouble(Harvester.DEPLOY_MM_CRUISE_RPS);
    return Math.max(0.0, Math.min(val, 6.0));
  }

  public void zeroDeployEncoder() {
    // Reset the encoder to 0.0 rotations.
    // The 0.05 is a timeout in seconds to ensure the CAN bus processes the request.
    deployMotor.setPosition(0.0, 0.05);
  }

  /** Returns the deploy motor stator current (in Amps) as reported by the motor controller. */
  public double getDeployStatorCurrent() {
    return deployMotor.getStatorCurrent().getValueAsDouble();
  }

  /** Convenience helper: true when absolute deploy stator current is >= the provided threshold. */
  public boolean isDeployCurrentAbove(double amps) {
    return Math.abs(getDeployStatorCurrent()) >= Math.abs(amps);
  }
}
