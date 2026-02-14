// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
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
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
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
    led = new LEDSubsystem();

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        vision = new VisionSubsystem(drive);

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        vision = new VisionSubsystem(drive);
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        vision = new VisionSubsystem(drive);

        break;
    }

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

    final JoystickButton flyWheel = new JoystickButton(operatorManualJoystick, 1);
    final JoystickButton kick = new JoystickButton(operatorManualJoystick, 2);
    final JoystickButton harvesterDeploy = new JoystickButton(operatorManualJoystick, 3);
    final JoystickButton harvesterSpin = new JoystickButton(operatorManualJoystick, 4);
    final JoystickButton indexerSpin = new JoystickButton(operatorManualJoystick, 5);

    final JoystickButton fire = new JoystickButton(operatorAutoJoystick, 1);
    final JoystickButton aim = new JoystickButton(operatorAutoJoystick, 2);
    final JoystickButton climberUp = new JoystickButton(operatorAutoJoystick, 9);
    final JoystickButton climberDown = new JoystickButton(operatorAutoJoystick, 10);

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

    // Shooter
    flyWheel.whileTrue(
        new Shoot(
            shooter,
            // Flywheel Speed: Scale Z axis from 0 to 1
            () -> (-operatorManualJoystick.getZ() + 1) / 2.0));

    // Kicker
    kick.whileTrue(new Kick(kicker));

    // Harvester Deploy
    harvesterDeploy.whileTrue(new HarvesterDeploy(harvester));

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
                .alongWith(new InstantCommand(() -> shooter.setTurretPosition(0.1)))
                .alongWith(new InstantCommand(() -> shooter.setHoodPosition((.1)))));

    // Prepare to shoot from Tower // TODO: TA - All parameters must be fixed
    new POVButton(operatorAutoJoystick, 180)
        .debounce(0.10)
        .onTrue(
            new Shoot(shooter, () -> (0.50))
                .alongWith(new InstantCommand(() -> shooter.setTurretPosition(0.1)))
                .alongWith(new InstantCommand(() -> shooter.setHoodPosition((.1)))));

    // Prepare to shoot from Depot // TODO: TA - All parameters must be fixed
    new POVButton(operatorAutoJoystick, 90)
        .debounce(0.10)
        .onTrue(
            new Shoot(shooter, () -> (0.50))
                .alongWith(new InstantCommand(() -> shooter.setTurretPosition(0.1)))
                .alongWith(new InstantCommand(() -> shooter.setHoodPosition((.1)))));

    // Prepare to shoot from Outpost // TODO: TA - All parameters must be fixed
    new POVButton(operatorAutoJoystick, 270)
        .debounce(0.10)
        .onTrue(
            new Shoot(shooter, () -> (0.50))
                .alongWith(new InstantCommand(() -> shooter.setTurretPosition(0.1)))
                .alongWith(new InstantCommand(() -> shooter.setHoodPosition((.1)))));

    fire.whileTrue(
        (new Kick(kicker)).alongWith(Commands.waitSeconds(0.2).andThen(new IndexerSpin(indexer))));

    // Moves to 4 rotations when button 9 is pressed
    climberUp.onTrue(new InstantCommand(() -> climber.setPosition(4.0), climber));

    // Returns to 0 rotations when button 10 is pressed
    climberDown.onTrue(new InstantCommand(() -> climber.setPosition(0.0), climber));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
