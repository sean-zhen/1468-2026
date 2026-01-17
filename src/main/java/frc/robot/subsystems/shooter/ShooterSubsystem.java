package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX flywheelLead = new TalonFX(20);
  private final TalonFX flywheelFollower = new TalonFX(21);
  private final TalonFX hoodMotor = new TalonFX(22);

  // Create separate requests for every motor
  private final DutyCycleOut leadRequest = new DutyCycleOut(0);
  private final DutyCycleOut followerRequest = new DutyCycleOut(0);
  private final DutyCycleOut hoodRequest = new DutyCycleOut(0);

  public ShooterSubsystem() {
    // Factory default configurations
    var config = new com.ctre.phoenix6.configs.TalonFXConfiguration();

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    flywheelLead.getConfigurator().apply(config);
    hoodMotor.getConfigurator().apply(config);

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    flywheelFollower.getConfigurator().apply(config);

    // Set Neutral Modes
    flywheelLead.setNeutralMode(NeutralModeValue.Coast);
    flywheelFollower.setNeutralMode(NeutralModeValue.Coast);
    hoodMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setFlywheel(double percent) {
    // Control both motors manually
    flywheelLead.setControl(leadRequest.withOutput(percent));
    flywheelFollower.setControl(followerRequest.withOutput(percent));
  }

  public void setHood(double percent) {
    hoodMotor.setControl(hoodRequest.withOutput(percent));
  }

  public void stop() {
    flywheelLead.stopMotor();
    flywheelFollower.stopMotor();
    hoodMotor.stopMotor();
  }
}
