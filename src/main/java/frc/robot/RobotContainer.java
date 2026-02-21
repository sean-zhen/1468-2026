// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.Constants.Harvester.*;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FaceTagsCommand;
import frc.robot.commands.HarvesterDeploy;
import frc.robot.commands.HarvesterSpin;
import frc.robot.commands.IndexerSpin;
import frc.robot.commands.Kick;
import frc.robot.commands.PrepareShooterCmd;
import frc.robot.commands.Shoot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.HarvesterSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final ShooterSubsystem shooter;
  private final VisionSubsystem vision;
  private final KickerSubsystem kicker;
  private final HarvesterSubsystem harvester;
  private final IndexerSubsystem indexer;
  private final ClimberSubsystem climber;
  private final LEDSubsystem led;

  // Controller
  final Joystick driverLeftJoystick = new Joystick(0);
  final Joystick driverRightJoystick = new Joystick(1);
  final Joystick operatorManualJoystick = new Joystick(2);
  final Joystick operatorAutoJoystick = new Joystick(3);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    shooter = new ShooterSubsystem();
    kicker = new KickerSubsystem();
    harvester = new HarvesterSubsystem();
    indexer = new IndexerSubsystem();
    climber = new ClimberSubsystem();

    drive =
        new Drive(
            new GyroIOPigeon2(),
            new ModuleIOTalonFX(TunerConstants.FrontLeft),
            new ModuleIOTalonFX(TunerConstants.FrontRight),
            new ModuleIOTalonFX(TunerConstants.BackLeft),
            new ModuleIOTalonFX(TunerConstants.BackRight));

    vision = new VisionSubsystem(drive);

    led = new LEDSubsystem(shooter, drive, vision);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // TODO: Decide on final button mappings
    final JoystickButton resetGyro = new JoystickButton(driverRightJoystick, 7);
    final JoystickButton lockToZero = new JoystickButton(driverRightJoystick, 9);
    final JoystickButton resetOdom = new JoystickButton(driverRightJoystick, 8);
    final JoystickButton faceHubButton = new JoystickButton(driverRightJoystick, 1);
    final JoystickButton turnLEDsOff = new JoystickButton(driverLeftJoystick, 11);

    final JoystickButton flyWheel = new JoystickButton(operatorManualJoystick, 1);
    final JoystickButton kickBtn = new JoystickButton(operatorManualJoystick, 2);
    final JoystickButton harvesterDeployBtn = new JoystickButton(operatorManualJoystick, 3);
    final JoystickButton harvesterSpin = new JoystickButton(operatorManualJoystick, 4);
    final JoystickButton indexerSpin = new JoystickButton(operatorManualJoystick, 5);

    final JoystickButton fire = new JoystickButton(operatorAutoJoystick, 1);
    final JoystickButton aim = new JoystickButton(operatorAutoJoystick, 2);
    final JoystickButton harvestStartBtn = new JoystickButton(operatorAutoJoystick, 5);
    final JoystickButton harvestStopBtn = new JoystickButton(operatorAutoJoystick, 3);
    final JoystickButton climberUpBtn = new JoystickButton(operatorAutoJoystick, 9);
    final JoystickButton climberDownBtn = new JoystickButton(operatorAutoJoystick, 10);
    final JoystickButton ManTurretBtn1 = new JoystickButton(operatorAutoJoystick, 11);
    final JoystickButton ManTurretBtn2 = new JoystickButton(operatorAutoJoystick, 12);

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverLeftJoystick.getY(),
            () -> -driverLeftJoystick.getX(),
            () -> -driverRightJoystick.getX()));

    // Lock to 0° when 9 on driver right controller  is held
    lockToZero.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -driverLeftJoystick.getY(),
            () -> -driverLeftJoystick.getX(),
            () -> new Rotation2d()));

    // Drive facing april tag
    faceHubButton.whileTrue(
        new FaceTagsCommand(
            drive, vision, () -> -driverLeftJoystick.getY(), () -> -driverLeftJoystick.getX()));

    // Reset gyro to 0° when 7 on right joystick button is pressed
    resetGyro.onTrue(
        Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                drive)
            .ignoringDisable(true));

    harvestStartBtn.onTrue(new HarvesterDeploy(harvester, DEPLOY_OUT_ANGLE, 0.0));
    harvestStopBtn.onTrue(new HarvesterDeploy(harvester, DEPLOY_IN_ANGLE, 0.0));

    // toggle LEDs on / off
    turnLEDsOff.onTrue(new InstantCommand(led::toggleLeds));

    ManTurretBtn1.onTrue(new InstantCommand(() -> shooter.setTurretPosition(0.330)));
    ManTurretBtn1.onFalse(new InstantCommand(() -> shooter.setTurretPosition(0.0)));
    ManTurretBtn2.onTrue(new InstantCommand(() -> shooter.setTurretPosition(-0.25)));
    ManTurretBtn2.onFalse(new InstantCommand(() -> shooter.setTurretPosition(-0.50)));

    // Shooter
    flyWheel.whileTrue(
        new Shoot(
            shooter,
            // Flywheel Speed: Scale Z axis from 0 to 1
            () -> (operatorManualJoystick.getZ() - 1) / 2.0));

    // Kicker
    kickBtn.whileTrue(new Kick(kicker));

    // Harvester Deploy
    // harvesterDeployBtn.onTrue(new HarvesterDeploy(harvester, DEPLOY_IN_ANGLE, 0.0));
    harvesterDeployBtn.onTrue(new HarvesterDeploy(harvester, 45.0, 0.0));
    harvesterDeployBtn.onFalse(new HarvesterDeploy(harvester, DEPLOY_START_ANGLE, 0.0));

    // Harvester Spin
    harvesterSpin.whileTrue(new HarvesterSpin(harvester));

    // Indexer Spin
    indexerSpin.whileTrue(new IndexerSpin(indexer));

    // Vision Reset
    resetOdom.onTrue(new InstantCommand(vision::forceOdometryToVision, vision));

    aim.whileTrue(new PrepareShooterCmd(shooter, drive));
    // fire.whileTrue(
    //     new PrepareShooterCmd(shooter, drive)
    //         .alongWith(Commands.waitSeconds(0.1).andThen(new Kick(kicker)))
    //         .alongWith(Commands.waitSeconds(0.2).andThen(new IndexerSpin(indexer))));

    // Prepare to shoot from Hub // TODO: TA - All parameters must be fixed
    new POVButton(operatorAutoJoystick, 0)
        .debounce(0.10)
        .onTrue(
            new Shoot(shooter, () -> (0.50))
                .alongWith(new InstantCommand(() -> shooter.setTurretPosition(0.0)))
                .alongWith(new InstantCommand(() -> shooter.setHoodPosition((.1)))));

    // Prepare to shoot from Tower // TODO: TA - All parameters must be fixed
    new POVButton(operatorAutoJoystick, 180)
        .debounce(0.10)
        .onTrue(
            new Shoot(shooter, () -> (0.50))
                .alongWith(new InstantCommand(() -> shooter.setTurretPosition(0.25)))
                .alongWith(new InstantCommand(() -> shooter.setHoodPosition((.1)))));

    // Prepare to shoot from Depot // TODO: TA - All parameters must be fixed
    new POVButton(operatorAutoJoystick, 90)
        .debounce(0.10)
        .onTrue(
            new Shoot(shooter, () -> (0.50))
                .alongWith(new InstantCommand(() -> shooter.setTurretPosition(0.5)))
                .alongWith(new InstantCommand(() -> shooter.setHoodPosition((.1)))));

    // Prepare to shoot from Outpost // TODO: TA - All parameters must be fixed
    new POVButton(operatorAutoJoystick, 270)
        .debounce(0.10)
        .onTrue(
            new Shoot(shooter, () -> (0.50))
                .alongWith(new InstantCommand(() -> shooter.setTurretPosition(-0.25)))
                .alongWith(new InstantCommand(() -> shooter.setHoodPosition((.1)))));

    fire.whileTrue(
        (new Kick(kicker)).alongWith(Commands.waitSeconds(0.2).andThen(new IndexerSpin(indexer))));

    // Moves to 4 rotations when button 9 is pressed
    climberUpBtn.onTrue(new InstantCommand(() -> climber.setPosition(40.0), climber));

    // Returns to 0 rotations when button 10 is pressed
    climberDownBtn.onTrue(new InstantCommand(() -> climber.setPosition(0.0), climber));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void updateGlobalHealth() {
    // 1. Get RoboRio CAN Health
    boolean shooterHealthy = shooter.isShooterConnected();
    boolean climberHealthy = climber.isClimberConnected();
    boolean kickerHealthy = kicker.isKickerConnected();
    boolean indexerHealthy = indexer.isIndexerConnected();
    boolean harvesterHealthy = harvester.isHarvesterConnected();
    boolean ledHealthy = led.isCanDleConnected();

    // 2. Get Swerve Health
    boolean canCoderHealthy = drive.isSwerveConnected();

    // 3. The Master Status
    boolean robotCanOk =
        harvesterHealthy
            && indexerHealthy
            && kickerHealthy
            && shooterHealthy
            && climberHealthy
            && ledHealthy;

    // Use a 2 "Master" widgets for the Main Dashboard
    SmartDashboard.putBoolean("RoboRio CAN STATUS", robotCanOk);
    SmartDashboard.putBoolean("CanCoder CAN STATUS", canCoderHealthy);

    // 1. Get RIO Bus Utilization
    var rioStatus = CANBus.roboRIO().getStatus();
    double rioUsage = rioStatus.BusUtilization * 100.0; // Convert 0.0-1.0 to %

    // 2. Get CANivore/CANcoder Bus Utilization
    // Replace "canivore1" with the name assigned in Phoenix Tuner X
    var canivoreStatus = new CANBus("1468_CANivore").getStatus();
    double canivoreUsage = canivoreStatus.BusUtilization * 100.0;

    // 3. Display on Dashboard
    SmartDashboard.putNumber("CAN RIO Usage %", rioUsage);
    SmartDashboard.putNumber("CAN CANivore Usage %", canivoreUsage);

    // Warning: FRC best practice is to keep utilization below 90%
    SmartDashboard.putBoolean("RIO CAN High Load Warning", rioUsage > 90);
    SmartDashboard.putBoolean("CANivore High Load Warning", canivoreUsage > 90);
  }
}
