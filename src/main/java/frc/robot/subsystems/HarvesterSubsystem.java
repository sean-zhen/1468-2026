package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  public HarvesterSubsystem() {
    // Deploy motor config (position/velocity, brake)
    var deployConfig = new com.ctre.phoenix6.configs.TalonFXConfiguration();
    var deploySlot0 = new Slot0Configs();
    deploySlot0.kP = Harvester.deploykP;
    deploySlot0.kI = Harvester.deploykI;
    deploySlot0.kD = Harvester.deploykD;
    deploySlot0.kV = Harvester.deploykV;
    deployConfig.Slot0 = deploySlot0;
    deployConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    deployMotor.getConfigurator().apply(deployConfig);
    deployMotor.setNeutralMode(NeutralModeValue.Brake);

    // Spin motor config (velocity, coast)
    var spinConfig = new com.ctre.phoenix6.configs.TalonFXConfiguration();
    var spinSlot0 = new Slot0Configs();
    spinSlot0.kP = Harvester.spinkP;
    spinSlot0.kI = Harvester.spinkI;
    spinSlot0.kD = Harvester.spinkD;
    spinSlot0.kV = Harvester.spinkV;
    spinConfig.Slot0 = spinSlot0;
    spinConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    spinMotor.getConfigurator().apply(spinConfig);
    spinMotor.setNeutralMode(NeutralModeValue.Coast);

    spinVelocitySignal = spinMotor.getVelocity();
    deployPositionSignal = deployMotor.getPosition();
  }

  @Override
  public void periodic() {
    log();
  }

  // Deploy position control (rotations at output)
  public void setDeployPosition(double rotations) {
    double motorRotations = rotations * Harvester.DEPLOY_GEAR_RATIO;
    deployMotor.setControl(deployPositionRequest.withPosition(motorRotations));
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

  public void log() {
    // Deploy motor logging
    SmartDashboard.putNumber(
        "Harv Deploy Pos (rot, output)",
        deployMotor.getPosition().getValueAsDouble() / Harvester.DEPLOY_GEAR_RATIO);
    SmartDashboard.putNumber(
        "Harv Deploy Velo (RPS, output)",
        deployMotor.getVelocity().getValueAsDouble() / Harvester.DEPLOY_GEAR_RATIO);
    SmartDashboard.putNumber("Harv Deploy Temp", deployMotor.getDeviceTemp().getValueAsDouble());

    // Spin motor logging
    SmartDashboard.putNumber(
        "Harv Spin Velo (RPS, output)",
        spinMotor.getVelocity().getValueAsDouble() / Harvester.SPIN_GEAR_RATIO);
    SmartDashboard.putNumber("Harv Spin Temp", spinMotor.getDeviceTemp().getValueAsDouble());

    boolean deployOK = deployPositionSignal.getStatus().isOK();
    SmartDashboard.putBoolean("Deply CAN OK", deployOK);
    boolean spinOK = spinVelocitySignal.getStatus().isOK();
    SmartDashboard.putBoolean("Spin CAN OK", spinOK);
  }

  public boolean isHarvesterConnected() {
    // Uses the checks we already built using getStatus().isOK()
    return spinVelocitySignal.getStatus().isOK() && deployPositionSignal.getStatus().isOK();
  }
}
