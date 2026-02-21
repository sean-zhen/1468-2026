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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Harvester;

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
            RotationsPerSecond.of(2)) // s/b 10 (mechanism) rotations per second cruise
        .withMotionMagicAcceleration(
            RotationsPerSecondPerSecond.of(
                2)) // s/b 4Take approximately 0.2 seconds to reach max vel
        // Take approximately 0.2 seconds to reach max accel
        .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(0));

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
      .add("Harv Deploy Pos (deg)", deployMotor.getPosition().getValueAsDouble() / DEPLOY_DEGREES_TO_ROTATIONS)
      .withWidget(BuiltInWidgets.kTextView)
      .withPosition(0, 0)
      .withSize(2, 1)
      .getEntry();
  harvDeployVeloEntry =
    harvTab
      .add("Harv Deploy Velo (RPS, output)", deployMotor.getVelocity().getValueAsDouble() / Harvester.DEPLOY_GEAR_RATIO)
      .withWidget(BuiltInWidgets.kTextView)
      .withPosition(2, 0)
      .withSize(2, 1)
      .getEntry();
  harvDeployTempEntry =
    harvTab.add("Harv Deploy Temp", deployMotor.getDeviceTemp().getValueAsDouble())
      .withWidget(BuiltInWidgets.kTextView)
      .withPosition(4, 0)
      .withSize(2, 1)
      .getEntry();

  harvSpinVeloEntry =
    harvTab
      .add("Harv Spin Velo (RPS, output)", spinMotor.getVelocity().getValueAsDouble() / Harvester.SPIN_GEAR_RATIO)
      .withWidget(BuiltInWidgets.kTextView)
      .withPosition(0, 1)
      .withSize(2, 1)
      .getEntry();
  harvSpinTempEntry =
    harvTab.add("Harv Spin Temp", spinMotor.getDeviceTemp().getValueAsDouble())
      .withWidget(BuiltInWidgets.kTextView)
      .withPosition(2, 1)
      .withSize(2, 1)
      .getEntry();

  var canTab = Shuffleboard.getTab("CAN Status");
  harvDeployCanOkEntry =
    canTab.add("Deply CAN OK", deployPositionSignal.getStatus().isOK())
      .withWidget(BuiltInWidgets.kBooleanBox)
      .withPosition(0, 5)
      .withSize(1, 1)
      .getEntry();
  harvSpinCanOkEntry =
    canTab.add("Spin CAN OK", spinVelocitySignal.getStatus().isOK())
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
  harvDeployPosEntry.setDouble(deployMotor.getPosition().getValueAsDouble() / DEPLOY_DEGREES_TO_ROTATIONS);
  harvDeployVeloEntry.setDouble(deployMotor.getVelocity().getValueAsDouble() / Harvester.DEPLOY_GEAR_RATIO);
  harvDeployTempEntry.setDouble(deployMotor.getDeviceTemp().getValueAsDouble());

  harvSpinVeloEntry.setDouble(spinMotor.getVelocity().getValueAsDouble() / Harvester.SPIN_GEAR_RATIO);
  harvSpinTempEntry.setDouble(spinMotor.getDeviceTemp().getValueAsDouble());

  boolean deployOK = deployPositionSignal.getStatus().isOK();
  boolean spinOK = spinVelocitySignal.getStatus().isOK();
  harvDeployCanOkEntry.setBoolean(deployOK);
  harvSpinCanOkEntry.setBoolean(spinOK);
  }

  public boolean isHarvesterConnected() {
    // Uses the checks we already built using getStatus().isOK()
    return spinVelocitySignal.getStatus().isOK() && deployPositionSignal.getStatus().isOK();
  }
}
