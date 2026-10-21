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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Harvester;

public class HarvesterSubsystem extends SubsystemBase {
  private final TalonFX deployMotor = new TalonFX(Harvester.DEPLOY_MOTOR_ID);

  // ✅ NEW
  private final TalonFX deployMotor2 =
      Harvester.USE_SECOND_DEPLOY_MOTOR ? new TalonFX(Harvester.DEPLOY_MOTOR_2_ID) : null;

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
  private final GenericEntry harvDeployHomingCurrentEntry;
  private final GenericEntry harvDeployHomingConsecEntry;
  private final GenericEntry harvDeployHomingTimeoutEntry;
  private final GenericEntry harvDeployCruiseRPSEntry;

  private final MotionMagicVoltage m_mmReqLt = new MotionMagicVoltage(0);

  private volatile boolean deployEndstopHit = false;

  public HarvesterSubsystem() {

    var deployConfig = new com.ctre.phoenix6.configs.TalonFXConfiguration();
    var deploySlot0 = new Slot0Configs();

    deploySlot0.kS = 0.25;
    deploySlot0.kV = 0.12;
    deploySlot0.kA = 0.01;
    deploySlot0.kP = 60;
    deploySlot0.kI = 0;
    deploySlot0.kD = 0.5;

    deployConfig.Slot0 = deploySlot0;
    deployConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    FeedbackConfigs fdb = deployConfig.Feedback;
    fdb.SensorToMechanismRatio = 25.0; // SWITCHED TO 25 ON 4/4/26

    setDeployPosition(DEPLOY_START_ANGLE);

    MotionMagicConfigs mm = deployConfig.MotionMagic;
    mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(DEPLOY_MM_CRUISE_RPS))
        .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(DEPLOY_MM_ACCEL_RPSPS))
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

    deployMotor.setNeutralMode(NeutralModeValue.Brake);

    // ✅ SECOND MOTOR CONFIG
    if (deployMotor2 != null) {
      var deployConfig2 = new com.ctre.phoenix6.configs.TalonFXConfiguration();
      deployConfig2.Slot0 = deployConfig.Slot0;
      deployConfig2.Feedback = deployConfig.Feedback;
      deployConfig2.MotionMagic = deployConfig.MotionMagic;
      deployConfig2.CurrentLimits = deployConfig.CurrentLimits;
      deployConfig2.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

      deployMotor2.getConfigurator().apply(deployConfig2);
      deployMotor2.setNeutralMode(NeutralModeValue.Brake);
    }

    // Spin config (unchanged)
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

    // --- ALL YOUR ORIGINAL SHUFFLEBOARD CODE UNCHANGED ---
    harvDeployPosEntry = harvTab.add("Harv Deploy Pos (deg)", 0).getEntry();
    harvDeployVeloEntry = harvTab.add("Harv Deploy Velo", 0).getEntry();
    harvDeployTempEntry = harvTab.add("Harv Deploy Temp", 0).getEntry();
    harvSpinVeloEntry = harvTab.add("Harv Spin Velo", 0).getEntry();
    harvSpinTempEntry = harvTab.add("Harv Spin Temp", 0).getEntry();
    harvDeployCurrentEntry = harvTab.add("Harv Deploy Current", 0).getEntry();
    harvDeployEndstopEntry = harvTab.add("Harv Deploy Endstop Hit", false).getEntry();

    harvDeployHomingCurrentEntry =
        harvTab.add("Deploy Homing Current (A)", Harvester.DEPLOY_HOMING_CURRENT_AMPS).getEntry();
    harvDeployHomingConsecEntry =
        harvTab.add("Deploy Homing Cycles", Harvester.DEPLOY_HOMING_CONSECUTIVE_CYCLES).getEntry();
    harvDeployHomingTimeoutEntry =
        harvTab.add("Deploy Timeout", Harvester.DEPLOY_HOMING_TIMEOUT_S).getEntry();
    harvDeployCruiseRPSEntry =
        harvTab.add("Deploy Cruise RPS", Harvester.DEPLOY_MM_CRUISE_RPS).getEntry();

    var canTab = Shuffleboard.getTab("CAN Status");
    harvDeployCanOkEntry = canTab.add("Deploy CAN OK", true).getEntry();
    harvSpinCanOkEntry = canTab.add("Spin CAN OK", true).getEntry();
  }

  @Override
  public void periodic() {
    log();
  }

  public void setCoastMode() {
    deployMotor.setNeutralMode(NeutralModeValue.Coast);
    if (deployMotor2 != null) deployMotor2.setNeutralMode(NeutralModeValue.Coast);
  }

  public void setBrakeMode() {
    deployMotor.setNeutralMode(NeutralModeValue.Brake);
    if (deployMotor2 != null) deployMotor2.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setHarvDeployMagicMoPos(double pos) {
    deployMotor.setControl(
        m_mmReqLt.withPosition(pos).withSlot(0).withOverrideBrakeDurNeutral(true));
    if (deployMotor2 != null)
      deployMotor2.setControl(
          m_mmReqLt.withPosition(pos).withSlot(0).withOverrideBrakeDurNeutral(true));
  }

  public double getDeployPositionDegrees() {
    return deployMotor.getPosition().getValueAsDouble() / DEPLOY_DEGREES_TO_ROTATIONS;
  }

  public void setDeployPosition(double degrees) {
    double shaftRotations = degrees * Harvester.DEPLOY_DEGREES_TO_ROTATIONS;
    deployMotor.setControl(deployPositionRequest.withPosition(shaftRotations));
    if (deployMotor2 != null)
      deployMotor2.setControl(deployPositionRequest.withPosition(shaftRotations));
  }

  public void setDeployVelocity(double rps) {
    double motorRPS = rps * Harvester.DEPLOY_GEAR_RATIO;
    deployMotor.setControl(deployVelocityRequest.withVelocity(motorRPS));
    if (deployMotor2 != null) deployMotor2.setControl(deployVelocityRequest.withVelocity(motorRPS));
  }

  public void setSpinVelocity(double rps) {
    double motorRPS = rps * Harvester.SPIN_GEAR_RATIO;
    spinMotor.setControl(spinVelocityRequest.withVelocity(motorRPS));
  }

  public void stopDeploy() {
    deployMotor.stopMotor();
    if (deployMotor2 != null) deployMotor2.stopMotor();
  }

  public void stopSpin() {
    spinMotor.stopMotor();
  }

  public void stopHarvester() {
    spinMotor.stopMotor();
    deployMotor.stopMotor();
    if (deployMotor2 != null) deployMotor2.stopMotor();
  }

  public void log() {
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

    harvDeployCanOkEntry.setBoolean(deployPositionSignal.getStatus().isOK());
    harvSpinCanOkEntry.setBoolean(spinVelocitySignal.getStatus().isOK());
  }

  public void setDeployEndstopHit(boolean hit) {
    deployEndstopHit = hit;
  }

  public boolean getDeployEndstopHit() {
    return deployEndstopHit;
  }

  public boolean isHarvesterConnected() {
    return spinVelocitySignal.getStatus().isOK() && deployPositionSignal.getStatus().isOK();
  }

  public void zeroDeployEncoder() {
    deployMotor.setPosition(0.0, 0.05);
  }

  public double getDeployStatorCurrent() {
    return deployMotor.getStatorCurrent().getValueAsDouble();
  }

  public boolean isDeployCurrentAbove(double amps) {
    return Math.abs(getDeployStatorCurrent()) >= Math.abs(amps);
  }

  public double getDeployHomingCurrentAmps() {
    double val = harvDeployHomingCurrentEntry.getDouble(Harvester.DEPLOY_HOMING_CURRENT_AMPS);
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
}
