package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.ClimbStateMachine.java.ClimbStateMachine;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlignToReefAuto;
import frc.robot.commands.ApproachReefPerpendicular;
import frc.robot.commands.AutoAlignToSource;
import frc.robot.commands.AutoPickupCoral;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeFromSourceParallel;
import frc.robot.commands.IntakingAlgaeParallel;
import frc.robot.commands.ReinitializingCommand;
import frc.robot.commands.ReleaseClawParallel;
// import frc.robot.commands.algaeintosource.ReleaseAlgae;
// import frc.robot.commands.algaeintoprocesser.AlgaeIntoProcesser;
import frc.robot.constants.FieldConstants;
// import frc.robot.commands.IntakeFromSource;
import frc.robot.constants.SimConstants;
import frc.robot.constants.SubsystemConstants.AlgaeState;
import frc.robot.constants.SubsystemConstants.CoralState;
import frc.robot.constants.SubsystemConstants.SuperStructureState;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.climber.ClimberArm;
import frc.robot.subsystems.climber.ClimberArmIO;
import frc.robot.subsystems.climber.ClimberArmIOSim;
import frc.robot.subsystems.climber.Winch;
import frc.robot.subsystems.climber.WinchIO;
import frc.robot.subsystems.climber.WinchIOSim;
import frc.robot.subsystems.commoniolayers.FlywheelIO;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LED_IO;
import frc.robot.subsystems.led.LED_IOCANdle;
import frc.robot.subsystems.led.LED_IOSim;
import frc.robot.subsystems.scoral.ScoralArm;
import frc.robot.subsystems.scoral.ScoralArmIOSim;
import frc.robot.subsystems.scoral.ScoralArmIOTalonFX;
import frc.robot.subsystems.scoral.ScoralRollers;
import frc.robot.subsystems.scoral.ScoralRollersIOSim;
import frc.robot.subsystems.scoral.ScoralSensorIO;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
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
  private final LED led;

  // Controller
  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController manipController = new CommandXboxController(1);
  //   private final Joystick joystikc = new Joystick(0);
  //   private final JoystickButton btn = new JoystickButton(joystikc, 4);
  //   private final KeyboardInputs keyboard = new KeyboardInputs(0);

  private ScoralArm scoralArm;
  // private final CoralScorerFlywheel coralIntake;

  public static Elevator elevator;
  public ClimberArm climberArm;
  private Vision vision;
  SuperStructure superStructure;
  private final Winch winch;
  private final SendableChooser<Command> autos;

  // public final Trigger elevatorBrakeTrigger;
  //   private final Trigger stateTrigger;
  // private final Trigger slowModeTrigger;

  private ScoralRollers scoralRollers;

  // public final Trigger elevatorBrakeTrigger;
  //   private final Trigger stateTrigger;
  private Trigger slowModeTrigger;
  private Trigger reefAlignTrigger;

  // Dashboard inputs
  private LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (SimConstants.currentMode) {
      case REAL:
        // // Real robot, instantiate hardware IO implementations
        // // TODO change lead, follower, gyro IDs, etc.
        // // elevator = new Elevator(new ElevatorIOTalonFX(8, 9, 0));
        // elevator = new Elevator(new ElevatorIOSim());
        // // winch = new Winch(new WinchIOTalonFX(12, 13));
        // winch = new Winch(new WinchIOSim());
        // // csFlywheel =
        // //     new CoralScorerFlywheel(
        // //         new CoralScorerFlywheelIOSim(),
        // //         new CoralSensorIO() {},
        // //         CoralState.DEFAULT,
        // //         AlgaeState.DEFAULT);
        // // led = new LED(new LED_IOCANdle(0, ""));
        // // superStructure = new SuperStructure(elevator, csArm, csFlywheel, drive, led);
        // drive =
        //     new Drive(
        //         new GyroIO() {},
        //         new ModuleIOSim(TunerConstants.FrontLeft),
        //         new ModuleIOSim(TunerConstants.FrontRight),
        //         new ModuleIOSim(TunerConstants.BackLeft),
        //         new ModuleIOSim(TunerConstants.BackRight));

        // csArm = new CoralScorerArm(new CoralScorerArmIOSim());

        // vision =
        //     new Vision(
        //         drive.getToPoseEstimatorConsumer(),
        //         new VisionIOLimelight("limelight 1", drive.getRawGyroRotationSupplier()),
        //         new VisionIOLimelight("limelight 2", drive.getRawGyroRotationSupplier()),
        //         new VisionIOLimelight("limelight 3", drive.getRawGyroRotationSupplier()),
        //         new VisionIOPhotonVision("photon", new Transform3d()));

        // csFlywheel =
        //     new CoralScorerFlywheel(
        //         new FlywheelIOTalonFX(),
        //         new CoralSensorIO() {},
        //         CoralState.DEFAULT,
        //         AlgaeState.DEFAULT);
        // led = new LED(new LED_IOCANdle(0, "CAN Bus 2"));
        // superStructure = new SuperStructure(drive, elevator, csArm, csFlywheel, led);
        // climberArm = new ClimberArm(new ClimberArmIOTalonFX(0, 0));

        elevator = new Elevator(new ElevatorIO() {});
        // winch = new Winch(new WinchIOTalonFX(12, 13));
        winch = new Winch(new WinchIO() {});
        // climberArm = new ClimberArm(new ClimberArmIOTalonFX(0, 0, 0));
        // csFlywheel =
        //     new CoralScorerFlywheel(
        //         new CoralScorerFlywheelIOSim(),
        //         new CoralSensorIO() {},
        //         CoralState.DEFAULT,
        //         AlgaeState.DEFAULT);
        // led = new LED(new LED_IOCANdle(0, ""));
        // superStructure = new SuperStructure(elevator, csArm, csFlywheel, drive, led);
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        scoralArm = new ScoralArm(new ScoralArmIOTalonFX(10, 19));

        vision =
            new Vision(
                drive.getToPoseEstimatorConsumer(),
                new VisionIOLimelight("limelight 1", drive.getRawGyroRotationSupplier()),
                new VisionIOLimelight("limelight 2", drive.getRawGyroRotationSupplier()),
                new VisionIOLimelight("limelight 3", drive.getRawGyroRotationSupplier()),
                new VisionIOPhotonVision("photon", new Transform3d()));

        // climberArm = new ClimberArm(new ClimberArmIOTalonFX(14, 5));
        climberArm = new ClimberArm(new ClimberArmIO() {});

        scoralRollers =
            new ScoralRollers(
                new ScoralRollersIOSim(),
                new ScoralSensorIO() {},
                CoralState.DEFAULT,
                AlgaeState.DEFAULT);
        led = new LED(new LED_IOCANdle(0, "CAN Bus 2"));

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

        scoralArm = new ScoralArm(new ScoralArmIOSim());
        winch = new Winch(new WinchIOSim());
        vision =
            new Vision(
                drive.getToPoseEstimatorConsumer(),
                new VisionIOLimelight("limelight 1", drive.getRawGyroRotationSupplier()),
                new VisionIOLimelight("limelight 2", drive.getRawGyroRotationSupplier()),
                new VisionIOLimelight("limelight 3", drive.getRawGyroRotationSupplier()),
                new VisionIOPhotonVision("photon", new Transform3d()));
        elevator = new Elevator(new ElevatorIOSim());
        scoralRollers =
            new ScoralRollers(
                new ScoralRollersIOSim(),
                new ScoralSensorIO() {},
                CoralState.DEFAULT,
                AlgaeState.DEFAULT);
        led = new LED(new LED_IOSim());
        superStructure = new SuperStructure(drive, elevator, scoralArm, scoralRollers, led);
        climberArm = new ClimberArm(new ClimberArmIOSim());
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

        scoralArm = new ScoralArm(new ScoralArmIOSim());
        // vision =
        //     new Vision(
        //         drive.getToPoseEstimatorConsumer(),
        //         new VisionIOLimelight("limelight 1", drive.getRawGyroRotationSupplier()),
        //         new VisionIOLimelight("limelight 2", drive.getRawGyroRotationSupplier()),
        //         new VisionIOLimelight("limelight 3", drive.getRawGyroRotationSupplier()),
        //         new VisionIOPhotonVision("photon", new Transform3d()));
        elevator = new Elevator(new ElevatorIO() {});
        scoralRollers =
            new ScoralRollers(
                new FlywheelIO() {},
                new ScoralSensorIO() {},
                CoralState.DEFAULT,
                AlgaeState.DEFAULT);
        led = new LED(new LED_IO() {});

        winch = new Winch(new WinchIO() {});

        superStructure = new SuperStructure(drive, elevator, scoralArm, scoralRollers, led);
        break;
    }

    // Set up auto routines
    // NamedCommands.registerCommand("AlignToReefAuto", new AlignToReefAuto(drive, led));

    // NamedCommands.registerCommand(
    //     "L1",
    //     new SequentialCommandGroup(
    //         new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L1)),
    //         new WaitUntilCommand(() -> superStructure.hasStructureReachedGoal)));
    // NamedCommands.registerCommand(
    //     "L2",
    //     new SequentialCommandGroup(
    //         new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L2)),
    //         new WaitUntilCommand(() -> superStructure.hasStructureReachedGoal())));
    // NamedCommands.registerCommand(
    //     "L3",
    //     new SequentialCommandGroup(
    //         new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L3)),
    //         new WaitUntilCommand(() -> superStructure.hasStructureReachedGoal())));
    // NamedCommands.registerCommand(
    //     "L4",
    //     new SequentialCommandGroup(
    //         new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L4)),
    //         new WaitUntilCommand(() -> superStructure.hasStructureReachedGoal())));
    // NamedCommands.registerCommand(
    //     "STOW ", new InstantCommand(() ->
    // superStructure.setWantedState(SuperStructureState.STOW)));
    // NamedCommands.registerCommand(
    //     "SCORE",
    //     new SequentialCommandGroup(
    //         new InstantCommand(
    //             () -> superStructure.setWantedState(SuperStructureState.SCORING_CORAL)),
    //         new WaitUntilCommand(() -> superStructure.hasStructureReachedGoal())));
    // NamedCommands.registerCommand(
    //     "INTAKE",
    //     new SequentialCommandGroup(
    //         new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.SOURCE)),
    //         new WaitUntilCommand(() -> superStructure.hasStructureReachedGoal())));

    // autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
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
    // autoChooser.addOption(
    //     "Top R3a", AutoBuilder.buildAuto("R3a(L3)-S1c-R2a(L3)-S2c-R1b(L3)-S3c-R6a(L3)"));
    // autoChooser.addDefaultOption("square", AutoBuilder.buildAuto("Square"));
    // autoChooser.addDefaultOption("1.1 auto", AutoBuilder.buildAuto("1.1.auto"));

    // Set up auto routines

    // Set up SysId routines

    NamedCommands.registerCommand(
        "ReleaseClawL1",
        new ReleaseClawParallel(FieldConstants.ReefHeight.L1, elevator, scoralArm, scoralRollers));
    NamedCommands.registerCommand(
        "ReleaseClawL2",
        new ReleaseClawParallel(FieldConstants.ReefHeight.L2, elevator, scoralArm, scoralRollers));
    NamedCommands.registerCommand(
        "ReleaseClawL3",
        new ReleaseClawParallel(FieldConstants.ReefHeight.L3, elevator, scoralArm, scoralRollers));
    NamedCommands.registerCommand(
        "ReleaseClawL4",
        new ReleaseClawParallel(FieldConstants.ReefHeight.L4, elevator, scoralArm, scoralRollers));

    NamedCommands.registerCommand("AlignToReefAuto", new AlignToReefAuto(drive, led));
    NamedCommands.registerCommand("AutoAlignToSource", new AutoAlignToSource(drive, led));
    NamedCommands.registerCommand(
        "IntakeFromSource",
        new IntakeFromSourceParallel(scoralRollers, scoralArm, elevator)
            .until(
                () ->
                    scoralRollers.seesCoral() == CoralState.SENSOR
                        || scoralRollers.seesCoral() == CoralState.CURRENT)
            .withTimeout(5));
    NamedCommands.registerCommand(
        "IntakingAlgae",
        new IntakingAlgaeParallel(elevator, scoralArm, scoralRollers)
            .until(() -> scoralRollers.seesAlgae() == AlgaeState.CURRENT)
            .withTimeout(5));
    // NamedCommands.registerCommand("Stow", new Stow(elevator, csArm));

    NamedCommands.registerCommand("AutoPickupCoral", new AutoPickupCoral(null, drive, led));

    autos = new SendableChooser<>();

    autos.addOption("MiddleLeft", AutoBuilder.buildAuto("BlueMiddleLeft"));
    autos.addOption("MiddleRight", AutoBuilder.buildAuto("BlueMiddleRight"));
    autos.addOption("Right", AutoBuilder.buildAuto("BlueRight"));
    autos.addOption("Left", AutoBuilder.buildAuto("BlueLeft"));
    autos.addOption("AutoSourceTop", AutoBuilder.buildAuto("Top-R5a(L4)-F2-R6b(L4)-F2-R6a(L4)"));
    autos.addOption("AutoTestTop", AutoBuilder.buildAuto("Top-R3b(L4)-F1-R2a(L4)-F1-R2b(L4)"));
    autos.addOption(
        "AutoTestMiddle", AutoBuilder.buildAuto("Middle-R3b(L4)-F1-R2a(L4)-F1-R2b(L4)"));
    autos.addOption(
        "AutoTestBottom", AutoBuilder.buildAuto("Bottom-R3b(L4)-F1-R2a(L4)-F1-R2b(L4)"));

    autoChooser.addOption(
      "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
     autoChooser.addOption(
         "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    //  autoChooser.addOption(
    //      "Drive SysId (Quasistatic Forward)",
    //      drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    //  autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addDefaultOption("square", AutoBuilder.buildAuto("Square"));
    // autoChooser.addOption("toReefTest", AutoBuilder.buildAuto("toReefTest"));

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", autos);
    // Configure the button bindings
    // configureButtonBindings();
    // stateTrigger = new Trigger(() -> superStructure.changedStated());
    // elevatorBrakeTrigger = new Trigger(() -> RobotController.getUserButton());
    slowModeTrigger = new Trigger(() -> superStructure.elevatorExtended());
    reefAlignTrigger =
        new Trigger(
            () ->
                driveController.leftTrigger().getAsBoolean()
                    || driveController.rightTrigger().getAsBoolean());
    // speedModeTrigger = new Trigger(() -> superStructure.elevatorExtended());
    configureButtonBindings();
    // test();
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void test() {
    // driveController.b().onTrue(elevator.setElevatorTarget(20, 1));
    // driveController.b().onFalse(elevator.setElevatorTarget(0, 1));
    //     driveController.a().onTrue(led.setStateCommand(LED_STATE.BLUE));
    //     driveController.y().onTrue(led.setStateCommand(LED_STATE.GREEN));
    //     driveController.x().onTrue(led.setStateCommand(LED_STATE.FIRE));
    // driveController.a().onTrue(winch.runVoltsCommmand(1));
    // driveController.a().onFalse(new InstantCommand(() -> winch.stop(), winch));
    // driveController.b().onTrue(climberArm.setArmTarget(20, 1));
    // driveController.b().onTrue(climberArm.setArmTarget(0, 1));

    driveController.a().onTrue(scoralArm.setArmTarget(20, 1));
    driveController.a().onFalse(new InstantCommand(() -> scoralArm.armStop()));
    driveController.b().onTrue(scoralArm.setArmTarget(0, 1));
    driveController.b().onFalse(new InstantCommand(() -> scoralArm.armStop()));
    // driveController.b().onTrue(new InstantCommand(() -> climberArm.armStop(), climberArm));
    // driveController
    //     .b()
    //     .onTrue(
    //         new ParallelCommandGroup(
    //             winch.runVoltsCommmand(2).until(() -> climberArm.getArmPositionDegs() == 130),
    //             climberArm.setArmTarget(130, 1)));
    // driveController
    //     .b()
    //     .onFalse(
    //         new ParallelCommandGroup(
    //             new InstantCommand(() -> winch.stop(), winch),
    //             new InstantCommand(() -> climberArm.armStop(), climberArm)));
  }

  private void configureButtonBindings() {
    slowModeTrigger.onTrue(new InstantCommand(() -> drive.enableSlowMode(true)));
    slowModeTrigger.onFalse(new InstantCommand(() -> drive.enableSlowMode(false)));

    driverControls();
    manipControls();
  }

  private void driverControls() {
    driveController
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            superStructure,
            led,
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX(),
            () -> driveController.leftBumper().getAsBoolean(),
            () -> driveController.leftTrigger().getAsBoolean(),
            () -> driveController.rightTrigger().getAsBoolean()));

    reefAlignTrigger.onFalse(
        new ConditionalCommand(
            new ApproachReefPerpendicular(drive, superStructure).withTimeout(2),
            new InstantCommand(),
            () -> (!drive.isNearReef() && drive.isAtReefSide())));

    driveController
        .rightBumper()
        .onTrue(
            new WaitUntilCommand(() -> superStructure.atGoals())
                .andThen(
                    new ReinitializingCommand(
                        () -> superStructure.getSuperStructureCommand(),
                        elevator,
                        scoralArm,
                        scoralRollers,
                        drive,
                        led))
                .andThen(new InstantCommand(() -> superStructure.advanceWantedState())));

    driveController
        .a()
        .onTrue(
            new InstantCommand(
                () -> superStructure.setWantedState(SuperStructureState.CLIMB_STAGE_ONE)));

    driveController
        .x()
        .onTrue(new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L3)));
    driveController
        .y()
        .onTrue(new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L4)));
    driveController
        .povUp()
        .onTrue(new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L2)));
    driveController
        .b()
        .onTrue(new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L1)));

    driveController
        .povUp()
        .onTrue(
            new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.SOURCE)));

    driveController
        .povLeft()
        .onTrue(
            new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.PROCESSOR)));

    driveController
        .povDown()
        .onTrue(
            new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.STOW))
                .andThen(
                    new ReinitializingCommand(
                        () -> superStructure.getSuperStructureCommand(),
                        elevator,
                        scoralArm,
                        scoralRollers,
                        drive,
                        led)));
  }

  private void manipControls() {
    manipController
        .x()
        .onTrue(new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L3)));
    manipController
        .y()
        .onTrue(new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L4)));
    manipController
        .a()
        .onTrue(new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L2)));
    manipController
        .b()
        .onTrue(new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L1)));

    manipController
        .povDown()
        .onTrue(
            new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.STOW))
                .andThen(
                    new ReinitializingCommand(
                        () -> superStructure.getSuperStructureCommand(),
                        elevator,
                        scoralArm,
                        scoralRollers,
                        drive,
                        led)));
  }

  //   private void testControls() {
  //     slowModeTrigger.onTrue(new InstantCommand(() -> drive.enableSlowMode(true)));
  //     slowModeTrigger.onFalse(new InstantCommand(() -> drive.enableSlowMode(false)));

  //     drive.setDefaultCommand(
  //         DriveCommands.joystickDrive(
  //             drive,
  //             superStructure,
  //             () -> -driveController.getLeftY(),
  //             () -> -driveController.getLeftX(),
  //             () -> -driveController.getRightX(),
  //             () -> driveController.leftBumper().getAsBoolean(),
  //             () -> driveController.leftTrigger().getAsBoolean(),
  //             () -> driveController.rightTrigger().getAsBoolean()));

  //     // Drive Controller - Align Commands go in Drive
  //     driveController.b().onTrue((new ReleaseAlgae(csFlywheel)));
  //     driveController.b().onFalse(new InstantCommand(() -> csFlywheel.runVolts(0)));

  //     // Manip Controller
  //     manipController.rightTrigger().onTrue(new Stow(elevator, csArm));

  //     manipController
  //         .leftBumper()
  //         .onTrue(
  //             new IntakeFromSourceParallel(csFlywheel, csArm, elevator)
  //                 .until(
  //                     () ->
  //                         csFlywheel.seesCoral() == CoralState.SENSOR
  //                             || csFlywheel.seesCoral() == CoralState.CURRENT)
  //                 .withTimeout(5));
  //     manipController
  //         .leftBumper()
  //         .onFalse(
  //             new ParallelCommandGroup(
  //                 new Stow(elevator, csArm), new InstantCommand(() -> csFlywheel.runVolts(0))));

  //     manipController
  //         .rightBumper()
  //         .onTrue(
  //             new IntakingAlgaeParallel(elevator, csArm, csFlywheel)
  //                 .until(() -> csFlywheel.seesAlgae() == AlgaeState.CURRENT)
  //                 .withTimeout(5));
  //     manipController
  //         .rightBumper()
  //         .onFalse(
  //             new ParallelCommandGroup(
  //                 new Stow(elevator, csArm), new InstantCommand(() -> csFlywheel.runVolts(0))));

  //     manipController
  //         .a()
  //         .onTrue(new ReleaseClawParallel(FieldConstants.ReefHeight.L1, elevator, csArm,
  // csFlywheel));
  //     manipController
  //         .a()
  //         .onFalse(
  //             new ParallelCommandGroup(
  //                 new Stow(elevator, csArm), new InstantCommand(() -> csFlywheel.runVolts(0))));

  //     manipController
  //         .b()
  //         .onTrue(new ReleaseClawParallel(FieldConstants.ReefHeight.L2, elevator, csArm,
  // csFlywheel));
  //     manipController
  //         .b()
  //         .onFalse(
  //             new ParallelCommandGroup(
  //                 new Stow(elevator, csArm), new InstantCommand(() -> csFlywheel.runVolts(0))));

  //     manipController
  //         .x()
  //         .onTrue(new ReleaseClawParallel(FieldConstants.ReefHeight.L3, elevator, csArm,
  // csFlywheel));
  //     manipController
  //         .x()
  //         .onFalse(
  //             new ParallelCommandGroup(
  //                 new Stow(elevator, csArm), new InstantCommand(() -> csFlywheel.runVolts(0))));

  //     manipController
  //         .y()
  //         .onTrue(new ReleaseClawParallel(FieldConstants.ReefHeight.L4, elevator, csArm,
  // csFlywheel));
  //     manipController
  //         .y()
  //         .onFalse(
  //             new ParallelCommandGroup(
  //                 new Stow(elevator, csArm), new InstantCommand(() -> csFlywheel.runVolts(0))));

  //     manipController
  //         .leftBumper()
  //         .onTrue(
  //             new ReinitializingCommand(
  //                 () -> superStructure.getSuperStructureCommand(), elevator, csArm, csFlywheel,
  // led));

  //     manipController.rightBumper().onFalse(new Stow(elevator, csArm));
  //   }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous.
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public ScoralArm getScoralArm() {
    return scoralArm;
  }

  public Drive getDrive() {
    return drive;
  }

  public Elevator getElevator() {
    return elevator;
  }

  public SuperStructure getSuperStructure() {
    return superStructure;
  }

  public LED getLED() {
    return led;
  }
}
