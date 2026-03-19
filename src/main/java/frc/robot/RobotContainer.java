// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.Constants.Harvester.*;
import static frc.robot.Constants.Rollers.*;
import static frc.robot.Constants.Shooter.*;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.AllMotorsBrake;
import frc.robot.commands.AllMotorsCoast;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToDepotCommandPP;
import frc.robot.commands.DriveToHubCommandPP;
import frc.robot.commands.HarvesterAgitate;
import frc.robot.commands.HarvesterDeploy;
import frc.robot.commands.HarvesterSpin;
import frc.robot.commands.Kick;
import frc.robot.commands.PrepareShooterCmd;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.HarvesterSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.util.Elastic;
import java.util.Set;
import java.util.function.DoubleSupplier;
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
  private final RollerSubsystem rollers;
  private final LEDSubsystem led;
  // Commands
  private final PrepareShooterCmd prepareShooterCmd;

  // Controller
  final Joystick driverLeftJoystick = new Joystick(0);
  final Joystick driverRightJoystick = new Joystick(1);
  final Joystick operatorManualJoystick = new Joystick(2);
  final Joystick operatorAutoJoystick = new Joystick(3);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  // CAN Status Shuffleboard entries
  private final ShuffleboardTab canTab = Shuffleboard.getTab("CAN Status");
  private final GenericEntry roboRioCanStatusEntry;
  private final GenericEntry canCoderCanStatusEntry;
  private final GenericEntry rioUsageEntry;
  private final GenericEntry canivoreUsageEntry;
  private final GenericEntry rioHighLoadEntry;
  private final GenericEntry canivoreHighLoadEntry;
  private final GenericEntry turretEntry;
  private final GenericEntry hoodEntry;
  private final GenericEntry flyEntry;

  // Class-level variables to track alert states
  private boolean canFaultAlertSent = false;
  private boolean canCoderFaultAlertSent = false;
  private boolean canUtilizationAlertSent = false;
  private boolean canCoderUtilizationAlertSent = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    shooter = new ShooterSubsystem();
    kicker = new KickerSubsystem();
    harvester = new HarvesterSubsystem();
    rollers = new RollerSubsystem();

    drive =
        new Drive(
            new GyroIOPigeon2(),
            new ModuleIOTalonFX(TunerConstants.FrontLeft),
            new ModuleIOTalonFX(TunerConstants.FrontRight),
            new ModuleIOTalonFX(TunerConstants.BackLeft),
            new ModuleIOTalonFX(TunerConstants.BackRight));

    vision = new VisionSubsystem(drive);

    led = new LEDSubsystem(shooter, drive, vision);

    prepareShooterCmd =
        new PrepareShooterCmd(
            shooter,
            drive,
            () -> DONT_OVERRIDE_VAL,
            () -> DONT_OVERRIDE_VAL,
            () -> DONT_OVERRIDE_VAL);

    // // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    /////////////////////////////////////////////////////////////////////////////////////////////
    // Register NamedCommands for PathPlanner Auto Routines
    // The drive commands are in the PathPlanner AutoRoutines only
    /////////////////////////////////////////////////////////////////////////////////////////////
    NamedCommands.registerCommand( // Only use in DEADLINE with a drive cmd as the DEADLINE
        "Aim",
        Commands.defer(
            () ->
                new PrepareShooterCmd(
                    shooter,
                    drive,
                    (DoubleSupplier) () -> DONT_OVERRIDE_VAL,
                    (DoubleSupplier) () -> DONT_OVERRIDE_VAL,
                    (DoubleSupplier) () -> DONT_OVERRIDE_VAL),
            Set.of(shooter)));

    NamedCommands.registerCommand( // Only use in DEADLINE with a drive cmd as the DEADLINE
        "Aim225",
        Commands.defer(
            () ->
                new PrepareShooterCmd(
                    shooter,
                    drive,
                    (DoubleSupplier) () -> 46.0,
                    (DoubleSupplier) () -> 0.30,
                    (DoubleSupplier) () -> DONT_OVERRIDE_VAL),
            Set.of(shooter)));

    NamedCommands.registerCommand( // Only use in DEADLINE with a drive cmd as the DEADLINE
        "Aim340",
        Commands.defer(
            () ->
                new PrepareShooterCmd(
                    shooter,
                    drive,
                    (DoubleSupplier) () -> 49.0,
                    (DoubleSupplier) () -> 0.51,
                    (DoubleSupplier) () -> DONT_OVERRIDE_VAL),
            Set.of(shooter)));

    NamedCommands.registerCommand( // Only use in DEADLINE with a drive cmd as the DEADLINE
        "HoodDown",
        Commands.defer(
            () ->
                new PrepareShooterCmd(
                    shooter,
                    drive,
                    (DoubleSupplier) () -> DONT_OVERRIDE_VAL,
                    (DoubleSupplier) () -> 0.0,
                    (DoubleSupplier) () -> DONT_OVERRIDE_VAL),
            Set.of(shooter)));

    NamedCommands.registerCommand(
        "Fire",
        Commands.defer(
            () ->
                new Kick(kicker)
                    .alongWith(
                        Commands.waitSeconds(0.2)
                            .andThen(
                                () -> new InstantCommand(() -> rollers.setVelocity(NORMAL_RPS)))),
            Set.of(kicker, rollers)));

    NamedCommands.registerCommand(
        "StopFire",
        Commands.defer(
            () ->
                new InstantCommand(() -> rollers.stop())
                    .andThen(
                        Commands.waitSeconds(0.2).andThen(new InstantCommand(() -> kicker.stop()))),
            Set.of(kicker, rollers)));

    NamedCommands.registerCommand(
        "StartHarvest",
        Commands.defer(
            () -> new HarvesterDeploy(harvester, DEPLOY_OUT_ANGLE, 0.0), Set.of(harvester)));

    NamedCommands.registerCommand(
        "StopHarvest",
        Commands.defer(
            () -> new HarvesterDeploy(harvester, DEPLOY_IN_ANGLE, 0.0), Set.of(harvester)));

    // ***** This must be after all NamedCommands are registered,
    // ***** as AutoBuilder.buildAutoChooser() needs to access them to build
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Initialize persistent CAN Status dashboard entries
    roboRioCanStatusEntry =
        canTab
            .add("RoboRio CAN STATUS", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(0, 0)
            .withSize(2, 1)
            .getEntry();
    canCoderCanStatusEntry =
        canTab
            .add("CanCoder CAN STATUS", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(2, 0)
            .withSize(2, 1)
            .getEntry();
    rioUsageEntry =
        canTab
            .add("CAN RIO Usage %", 0.0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withPosition(0, 1)
            .withSize(3, 1)
            .getEntry();
    canivoreUsageEntry =
        canTab
            .add("CAN CANivore Usage %", 0.0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withPosition(3, 1)
            .withSize(3, 1)
            .getEntry();
    rioHighLoadEntry =
        canTab
            .add("RIO CAN High Load Warning", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(0, 2)
            .withSize(2, 1)
            .getEntry();
    canivoreHighLoadEntry =
        canTab
            .add("CANivore High Load Warning", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(2, 2)
            .withSize(2, 1)
            .getEntry();

    // 1. Get the Tab
    ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
    // --- ROW 0: INSTRUCTIONAL LABELS (Width 3) ---
    // shooterTab.add("Flywheel RPS", "0-100 [999 = Auto]").withPosition(0, 0).withSize(3, 1);

    // shooterTab.add("Hood Deg", "(0-20°) [999 = Auto]").withPosition(3, 0).withSize(3, 1);

    // shooterTab.add("Turret Deg", "(±180°) [999 = Auto]").withPosition(6, 0).withSize(3, 1);

    // --- ROW 1: INPUT BOXES (Width 3) ---
    flyEntry =
        shooterTab
            .add("Flywheel OverRide", 999.0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 1)
            .withSize(3, 1)
            .getEntry();

    hoodEntry =
        shooterTab
            .add("Hood OverRide", 999.0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(3, 1)
            .withSize(3, 1)
            .getEntry();

    turretEntry =
        shooterTab
            .add("Turret OverRide", 999.0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(6, 1)
            .withSize(3, 1)
            .getEntry();

    // --- THE RESET BUTTON ---
    // This creates a button on the dashboard that sets all values back to 999
    shooterTab
        .add(
            "RESET ALL",
            Commands.runOnce(
                () -> {
                  flyEntry.setDouble(999.0);
                  hoodEntry.setDouble(999.0);
                  turretEntry.setDouble(999.0);
                }))
        .withWidget(BuiltInWidgets.kCommand)
        .withPosition(9, 0)
        .withSize(2, 2); // A tall 2x2 button next to the inputs

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
    ////////////////////////////////////////////////////////////////////////////////////////////
    /// Driver Left Buttons (Joystick 0)
    ////////////////////////////////////////////////////////////////////////////////////////////
    final JoystickButton stopWithXPatBtn = new JoystickButton(driverLeftJoystick, 1);
    final JoystickButton driveToDepot = new JoystickButton(driverLeftJoystick, 3);
    final JoystickButton driveToOutpost = new JoystickButton(driverLeftJoystick, 4);
    final JoystickButton driveToHub = new JoystickButton(driverLeftJoystick, 5);
    final JoystickButton driveToTower = new JoystickButton(driverLeftJoystick, 6);
    final JoystickButton turnLEDsOff = new JoystickButton(driverLeftJoystick, 11);
    ////////////////////////////////////////////////////////////////////////////////////////////
    /// Driver Right Buttons (Joystick 1)
    ////////////////////////////////////////////////////////////////////////////////////////////
    final JoystickButton faceHubButton = new JoystickButton(driverRightJoystick, 1);
    final JoystickButton lockToTargetBtn = new JoystickButton(driverRightJoystick, 2);
    final JoystickButton resetGyro = new JoystickButton(driverRightJoystick, 7);
    final JoystickButton resetTurret = new JoystickButton(driverRightJoystick, 11);

    // final JoystickButton resetOdom = new JoystickButton(driverRightJoystick, 8);
    ////////////////////////////////////////////////////////////////////////////////////////////
    /// Operator Left Buttons (Joystick 2) Manual / Override Buttons
    ////////////////////////////////////////////////////////////////////////////////////////////
    final JoystickButton flyWheel = new JoystickButton(operatorManualJoystick, 1);
    final JoystickButton kickBtn = new JoystickButton(operatorManualJoystick, 2);
    final JoystickButton harvesterDeployBtn = new JoystickButton(operatorManualJoystick, 3);
    final JoystickButton harvesterSpin = new JoystickButton(operatorManualJoystick, 4);
    final JoystickButton rollersSpin = new JoystickButton(operatorManualJoystick, 5);
    final JoystickButton harvesterSpinReverse = new JoystickButton(operatorManualJoystick, 6);
    final JoystickButton rollersSpinReverse = new JoystickButton(operatorManualJoystick, 7);
    final JoystickButton zeroHoodBtn = new JoystickButton(operatorManualJoystick, 8);
    final JoystickButton zeroTurretBtn = new JoystickButton(operatorManualJoystick, 9);
    final JoystickButton resetHarvesterEncoder = new JoystickButton(operatorManualJoystick, 10);
    final JoystickButton manPrepareShtrBtn2 = new JoystickButton(operatorManualJoystick, 11);
    ////////////////////////////////////////////////////////////////////////////////////////////
    /// Operator Right Buttons (Joystick 3) Automatic / Real Game Operation
    ////////////////////////////////////////////////////////////////////////////////////////////
    final JoystickButton fireBtn = new JoystickButton(operatorAutoJoystick, 1);
    final JoystickButton aimBtn = new JoystickButton(operatorAutoJoystick, 2);
    final JoystickButton harvestStopBtn = new JoystickButton(operatorAutoJoystick, 3);
    final JoystickButton harvestStartBtn = new JoystickButton(operatorAutoJoystick, 5);
    final JoystickButton harvestAgitate = new JoystickButton(operatorAutoJoystick, 6);
    final JoystickButton aimStopBtn = new JoystickButton(operatorAutoJoystick, 8);
    final JoystickButton climberUpBtn = new JoystickButton(operatorAutoJoystick, 9);
    final JoystickButton climberDownBtn = new JoystickButton(operatorAutoJoystick, 10);
    ////////////////////////////////////////////////////////////////////////////////////////////

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverLeftJoystick.getY(),
            () -> -driverLeftJoystick.getX(),
            () -> -driverRightJoystick.getX()));

    //////////////////////////////////////////////////////////////
    /// Driver Buttons / Commands Left Joystick (0)
    //////////////////////////////////////////////////////////////

    // Switch to X pattern
    stopWithXPatBtn.onTrue(Commands.runOnce(drive::stopWithX, drive));

    driveToDepot.onTrue(DriveToDepotCommandPP.create(drive));
    driveToOutpost.onTrue(DriveToDepotCommandPP.create(drive));
    driveToHub.onTrue(DriveToHubCommandPP.create(drive));
    // driveToTower.onTrue(DriveToTowerCommandPP.create(drive));

    // Stop! - This will stop a "on the fly" pathplanner path
    new JoystickButton(driverLeftJoystick, 9).onTrue(Commands.runOnce(drive::stop, drive));

    // toggle LEDs on / off
    turnLEDsOff.onTrue(new InstantCommand(led::toggleLeds));

    //////////////////////////////////////////////////////////////
    /// Driver Buttons / Commands Right Joystick (1)
    //////////////////////////////////////////////////////////////

    // Lock Onto Hub
    // While holding Button 1 on the Right Joystick, lock heading to the Hub
    // faceHubButton.whileTrue(
    //     DriveLockToHubCmd.create(
    //         drive, () -> -driverLeftJoystick.getY(), () -> -driverLeftJoystick.getX()));

    // Supplier<Rotation2d> sweepRotationSupplier =
    //     () -> {
    //       // 1. Get the base vision angle
    //       double targetAngle = shooter.getAngleToHub(drive.getPose()).getDegrees();

    //       // 2. Determine strafe direction (X on the field)
    //       double strafeVelocity = -driverLeftJoystick.getX();

    //       // 3. Determine Alliance (Red is flipped)
    //       boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

    //       // 4. Calculate Offset
    //       // If moving "Right" relative to your perspective, add/sub 90
    //       // to make the front of the robot lead the movement.
    //       double offset = 0;
    //       if (Math.abs(strafeVelocity) > 0.1) {
    //         if (isRed) {
    //           offset = (strafeVelocity > 0) ? -90 : 90;
    //         } else {
    //           offset = (strafeVelocity > 0) ? 90 : -90;
    //         }
    //       }

    //       return Rotation2d.fromDegrees(targetAngle + offset);
    //     };

    // Lock to Hub / target while button is held
    faceHubButton.whileTrue(
        // DriveCommands.joystickDriveAtAngle(
        //     drive,
        //     () -> -driverLeftJoystick.getY(),
        //     () -> -driverLeftJoystick.getX(),
        //     sweepRotationSupplier));
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -driverLeftJoystick.getY(),
            () -> -driverLeftJoystick.getX(),
            () -> prepareShooterCmd.getAimAngle(),
            () -> prepareShooterCmd.getVirtualTarget()));

    // Lock to Hub / target while button is held
    lockToTargetBtn.whileTrue(
        // DriveCommands.joystickDriveAtAngle(
        //     drive,
        //     () -> -driverLeftJoystick.getY(),
        //     () -> -driverLeftJoystick.getX(),
        //     sweepRotationSupplier));
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -driverLeftJoystick.getY(),
            () -> -driverLeftJoystick.getX(),
            () -> prepareShooterCmd.getAimAngle(),
            () -> prepareShooterCmd.getVirtualTarget()));

    // Reset gyro to 0° when 7 on right joystick button is pressed
    resetGyro.onTrue(
        Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                drive)
            .ignoringDisable(true));

    new JoystickButton(driverRightJoystick, 10)
        .onTrue(new AllMotorsBrake(harvester, shooter, rollers, kicker, drive));
    new JoystickButton(driverRightJoystick, 11)
        .onTrue(new AllMotorsCoast(harvester, shooter, rollers, kicker, drive));

    // Reset Turret to 0

    // resetTurret.onTrue(
    //     Command.runOnce
    // )

    //////////////////////////////////////////////////////////////
    /// Operator Automatic Buttons / Commands (Joystick 3 / Right Hand)
    //////////////////////////////////////////////////////////////

    // Start up Kicker to get it up to speed, alongWith getting the hood up (also insures flywheel
    // up to speed and angle to target is calculated)
    // and then start rollers which will start the actual shooting of the balls
    // kicker
    fireBtn
        .onTrue(new InstantCommand(() -> prepareShooterCmd.setShootingMode(true)))
        .onFalse(new InstantCommand(() -> prepareShooterCmd.setShootingMode(false)));

    fireBtn.whileTrue(
        new Kick(kicker)
            .alongWith(prepareShooterCmd)
            .alongWith(
                Commands.waitSeconds(0.2)
                    .andThen(new InstantCommand(() -> rollers.setVelocity(NORMAL_RPS)))));
    //     .onlyIf(() -> prepareShooterCmd.isAligned()));

    // TODO: TA - add the onlyif logic when tested that everything works without it.
    // fireBtn.whileTrue(
    //     new Kick(kicker)
    //         .alongWith(
    //             new PrepareShooterCmd(
    //                 shooter,
    //                 drive,
    //                 () -> DONT_OVERRIDE_VAL,
    //                 () -> DONT_OVERRIDE_VAL,
    //                 () -> DONT_OVERRIDE_VAL))
    //         .alongWith(
    //             Commands.waitSeconds(0.2)
    //                 .andThen(new InstantCommand(() -> rollers.setVelocity(NORMAL_RPS))))
    //         .onlyIf(() -> prepareShooterCmd.isAligned()));

    // letting go of the button, willstop shooting by first stopping the rollers, and
    fireBtn
        .debounce(0.10)
        .onFalse(
            new InstantCommand(() -> rollers.stop())
                .andThen(
                    Commands.waitSeconds(0.2)
                        .andThen(new InstantCommand(() -> kicker.stop()))
                        .alongWith(
                            new PrepareShooterCmd(
                                shooter,
                                drive,
                                (DoubleSupplier) () -> DONT_OVERRIDE_VAL,
                                (DoubleSupplier) () -> 0.0,
                                (DoubleSupplier) () -> DONT_OVERRIDE_VAL))));

    // Start preparing flyywheel when Aim button is pressed and keep running until interrupted
    // Keep hood at 0.0 for safety
    aimBtn.onTrue(
        Commands.runOnce(() -> prepareShooterCmd.setShootingMode(false))
            .andThen(prepareShooterCmd));

    // Stop aiming and reset shooter outputs when Aim Stop button is pressed
    aimStopBtn.onTrue(
        new PrepareShooterCmd(
            shooter,
            drive,
            (DoubleSupplier) () -> 0.0,
            (DoubleSupplier) () -> 0.0,
            (DoubleSupplier) () -> 0.0));

    harvestStartBtn.onTrue(new HarvesterDeploy(harvester, DEPLOY_OUT_ANGLE, 0.0));
    harvestStopBtn.onTrue(new HarvesterDeploy(harvester, DEPLOY_IN_ANGLE, 0.0));

    harvestAgitate.whileTrue(HarvesterAgitate.create(harvester));

    // Prepare to shoot from Hub // TODO: TA - All parameters must be fixed
    new POVButton(operatorAutoJoystick, 0)
        .debounce(0.10)
        .onTrue(
            new PrepareShooterCmd(
                shooter,
                drive,
                (DoubleSupplier) () -> DONT_OVERRIDE_VAL,
                (DoubleSupplier) () -> DONT_OVERRIDE_VAL,
                (DoubleSupplier) () -> 0.0));

    // Prepare to shoot from Tower // TODO: TA - All parameters must be fixed
    new POVButton(operatorAutoJoystick, 180)
        .debounce(0.10)
        .onTrue(
            new PrepareShooterCmd(
                shooter,
                drive,
                (DoubleSupplier) () -> DONT_OVERRIDE_VAL,
                (DoubleSupplier) () -> DONT_OVERRIDE_VAL,
                (DoubleSupplier) () -> 0.0));

    // Prepare to shoot from Depot // TODO: TA - All parameters must be fixed
    new POVButton(operatorAutoJoystick, 90)
        .debounce(0.10)
        .onTrue(
            new PrepareShooterCmd(
                shooter,
                drive,
                (DoubleSupplier) () -> DONT_OVERRIDE_VAL,
                (DoubleSupplier) () -> DONT_OVERRIDE_VAL,
                (DoubleSupplier) () -> -90.0));

    // Prepare to shoot from Outpost // TODO: TA - All parameters must be fixed
    new POVButton(operatorAutoJoystick, 270)
        .debounce(0.10)
        .onTrue(
            new PrepareShooterCmd(
                shooter,
                drive,
                (DoubleSupplier) () -> DONT_OVERRIDE_VAL,
                (DoubleSupplier) () -> DONT_OVERRIDE_VAL,
                (DoubleSupplier) () -> 90.0));

    //////////////////////////////////////////////////////////////
    /// Operator Manual Buttons / Commands (Joystick 2 / Left Hand)
    //////////////////////////////////////////////////////////////

    flyWheel
        .debounce(0.10)
        .onTrue(
            (new PrepareShooterCmd(
                shooter,
                drive,
                (DoubleSupplier) () -> (operatorManualJoystick.getZ() - 1) / 2.0,
                (DoubleSupplier) () -> hoodEntry.getDouble(999.0),
                (DoubleSupplier) () -> turretEntry.getDouble(999.0))));

    // Kicker
    kickBtn.whileTrue(new Kick(kicker));

    // Harvester Deploy
    // harvesterDeployBtn.onTrue(new HarvesterDeploy(harvester, DEPLOY_IN_ANGLE, 0.0));
    harvesterDeployBtn.onTrue(new HarvesterDeploy(harvester, 45.0, 0.0));
    harvesterDeployBtn.onFalse(new HarvesterDeploy(harvester, DEPLOY_START_ANGLE, 0.0));

    // Harvester Spin
    harvesterSpin.whileTrue(new HarvesterSpin(harvester, false));
    harvesterSpinReverse.whileTrue(new HarvesterSpin(harvester, true));

    // rollers Spin
    rollersSpin.whileTrue(new InstantCommand(() -> rollers.setVelocity(NORMAL_RPS)));
    rollersSpinReverse.whileTrue(new InstantCommand(() -> rollers.setVelocity(REVERSE_RPS)));

    zeroHoodBtn
        .debounce(0.10)
        .onTrue(
            (new PrepareShooterCmd(
                shooter,
                drive,
                (DoubleSupplier) () -> flyEntry.getDouble(999.0),
                (DoubleSupplier) () -> 0.0,
                (DoubleSupplier) () -> turretEntry.getDouble(999.0))));

    zeroTurretBtn
        .debounce(0.10)
        .onTrue(
            (new PrepareShooterCmd(
                shooter,
                drive,
                (DoubleSupplier) () -> flyEntry.getDouble(999.0),
                (DoubleSupplier) () -> hoodEntry.getDouble(999.0),
                (DoubleSupplier) () -> 0.0)));

    resetHarvesterEncoder.onTrue(new InstantCommand(() -> harvester.zeroDeployEncoder()));

    // flyEntry = Shuffleboard.getTab("Shooter").add("Flywheel", 0).getEntry();
    // hoodEntry = Shuffleboard.getTab("Shooter").add("Hood", 0).getEntry();
    // turretEntry = Shuffleboard.getTab("Shooter").add("Turret", 0).getEntry();
    manPrepareShtrBtn2.onTrue(
        (new PrepareShooterCmd(
            shooter,
            drive,
            (DoubleSupplier) () -> flyEntry.getDouble(999.0),
            (DoubleSupplier) () -> hoodEntry.getDouble(999.0),
            (DoubleSupplier) () -> turretEntry.getDouble(999.0))));
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
    // ── Grab the CAN Status Shuffleboard tab ──────────────────────────────────

    // 1. Get RoboRio CAN Health
    boolean shooterHealthy = shooter.isShooterConnected();
    boolean kickerHealthy = kicker.isKickerConnected();
    boolean rollersHealthy = rollers.isRollerConnected();
    boolean harvesterHealthy = harvester.isHarvesterConnected();
    boolean ledHealthy = led.isCanDleConnected();

    // 2. Get Swerve Health
    boolean canCoderHealthy = drive.isSwerveConnected();

    // 3. The Master Status
    boolean robotCanOk =
        harvesterHealthy && rollersHealthy && kickerHealthy && shooterHealthy && ledHealthy;

    // Update master CAN indicators on the CAN Status tab
    roboRioCanStatusEntry.setBoolean(robotCanOk);
    canCoderCanStatusEntry.setBoolean(canCoderHealthy);

    // 4. Get RIO Bus Utilization
    var rioStatus = CANBus.roboRIO().getStatus();
    double rioUsage = rioStatus.BusUtilization * 100.0; // Convert 0.0-1.0 to %

    // 5. Get CANivore/CANcoder Bus Utilization
    // Replace "canivore1" with the name assigned in Phoenix Tuner X
    var canivoreStatus = new CANBus("1468_CANivore").getStatus();
    double canivoreUsage = canivoreStatus.BusUtilization * 100.0;

    // 6. Display bus utilization on the CAN Status tab
    rioUsageEntry.setDouble(rioUsage);
    canivoreUsageEntry.setDouble(canivoreUsage);

    // 7. High-load warnings (>90% is FRC best practice threshold)
    rioHighLoadEntry.setBoolean(rioUsage > 90);
    canivoreHighLoadEntry.setBoolean(canivoreUsage > 90);

    if (!robotCanOk) {
      if (!canFaultAlertSent) {
        Elastic.sendNotification(
            new Elastic.Notification(
                Elastic.NotificationLevel.ERROR,
                "CAN BUS FAULT",
                "One or more RoboRIO CAN devices are disconnected! Check shooter, harvester, rollers, kicker, climber, or LED."));
        canFaultAlertSent = true; // Mark as sent so it doesn't repeat
      }
    } else {
      // Reset the alert state when the CAN bus is healthy again
      canFaultAlertSent = false;
    }

    if (!canCoderHealthy) {
      if (!canCoderFaultAlertSent) {
        Elastic.sendNotification(
            new Elastic.Notification(
                Elastic.NotificationLevel.ERROR,
                "Swerve CANcoder Fault",
                "One or more swerve CANcoders are disconnected!"));
        canCoderFaultAlertSent = true; // Mark as sent so it doesn't repeat
      }
    } else {
      // Reset the alert state when the CAN bus is healthy again
      canCoderFaultAlertSent = false;
    }

    if (rioUsage > 90) {
      if (!canUtilizationAlertSent) {
        Elastic.sendNotification(
            new Elastic.Notification(
                Elastic.NotificationLevel.ERROR,
                "RIO CAN High Load",
                "RIO CAN bus at %.1f%% utilization — keep below 90%"));
        canUtilizationAlertSent = true; // Mark as sent so it doesn't repeat
      }
    } else {
      // Reset the alert state when the CAN bus is healthy again
      canUtilizationAlertSent = false;
    }

    if (canivoreUsage > 90) {
      if (!canCoderUtilizationAlertSent) {
        Elastic.sendNotification(
            new Elastic.Notification(
                Elastic.NotificationLevel.ERROR,
                "CANivore High Load",
                "CANivore bus at %.1f%% utilization — keep below 90%%.\", canivoreUsage"));
        canCoderUtilizationAlertSent = true; // Mark as sent so it doesn't repeat
      }
    } else {
      // Reset the alert state when the CAN bus is healthy again
      canCoderUtilizationAlertSent = false;
    }
  }
}
