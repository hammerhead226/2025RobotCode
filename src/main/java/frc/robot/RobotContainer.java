package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.ClimbStateMachine.java.ClimbStateMachine;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ApproachReefPerpendicular;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeFromSourceParallel;
import frc.robot.commands.IntakingAlgaeParallel;
import frc.robot.commands.ReinitializingCommand;
import frc.robot.commands.ReleaseClawParallel;
import frc.robot.commands.Stow;
import frc.robot.commands.algaeintoprocesser.ReleaseAlgae;
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
import frc.robot.subsystems.climber.ClimberArmIOSim;
import frc.robot.subsystems.climber.ClimberArmIOTalonFX;
import frc.robot.subsystems.climber.Winch;
import frc.robot.subsystems.climber.WinchIO;
import frc.robot.subsystems.climber.WinchIOSim;
import frc.robot.subsystems.climber.WinchIOTalonFX;
import frc.robot.subsystems.commoniolayers.FlywheelIO;
import frc.robot.subsystems.coralscorer.CoralScorerArm;
import frc.robot.subsystems.coralscorer.CoralScorerArmIOSim;
import frc.robot.subsystems.coralscorer.CoralScorerFlywheel;
import frc.robot.subsystems.coralscorer.CoralSensorIO;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LED_IO;
import frc.robot.subsystems.led.LED_IOCANdle;
import frc.robot.subsystems.led.LED_IOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import java.util.Map;
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

  // Autos
  private final SendableChooser autos;

  // Controller
  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController manipController = new CommandXboxController(1);
  //   private final Joystick joystikc = new Joystick(0);
  //   private final JoystickButton btn = new JoystickButton(joystikc, 4);
  //   private final KeyboardInputs keyboard = new KeyboardInputs(0);

  private CoralScorerArm csArm;
  // private final CoralScorerFlywheel coralIntake;

  public static Elevator elevator;
  public ClimberArm climberArm;
  private Vision vision;
  SuperStructure superStructure;
  private final Winch winch;

  // public final Trigger elevatorBrakeTrigger;
  //   private final Trigger stateTrigger;
  // private final Trigger slowModeTrigger;

  private Trigger stateTrigger;

  private CoralScorerFlywheel csFlywheel;

  private Command superStructureCommands;

  private SuperStructureState stateSelect() {
    return superStructure.getWantedState();
  }
  // public final Trigger elevatorBrakeTrigger;
  //   private final Trigger stateTrigger;
  //   private Trigger slowModeTrigger;

  // Dashboard inputs
  private LoggedDashboardChooser<Command> autoChooser;

  //   private CLIMB_STATES climbSelect() {
  //     return climbStateMachine.getTargetState();
  //   }

  private Command climbCommands;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (SimConstants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // drive =
        //     new Drive(
        //         new GyroIOPigeon2(),
        //         new ModuleIOTalonFX(TunerConstants.FrontLeft),
        //         new ModuleIOTalonFX(TunerConstants.FrontRight),
        //         new ModuleIOTalonFX(TunerConstants.BackLeft),
        //         new ModuleIOTalonFX(TunerConstants.BackRight));

        //  csArm = new CoralScorerArm(new CoralScorerArmIOTalonFX(1));

        // vision =
        //     new Vision(
        //         drive.getToPoseEstimatorConsumer(),
        //         new VisionIOLimelight("limelight 1", drive.getRawGyroRotationSupplier()),
        //         new VisionIOLimelight("limelight 2", drive.getRawGyroRotationSupplier()),
        //         new VisionIOLimelight("limelight 3", drive.getRawGyroRotationSupplier()),
        //         new VisionIOPhotonVision("photon", new Transform3d()));
        // TODO change lead, follower, gyro IDs, etc.
        // elevator = new Elevator(new ElevatorIOTalonFX(8, 9, 0));
        elevator = new Elevator(new ElevatorIOSim());
        // winch = new Winch(new WinchIOTalonFX(12, 13));
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
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        csArm = new CoralScorerArm(new CoralScorerArmIOSim());

        vision =
            new Vision(
                drive.getToPoseEstimatorConsumer(),
                new VisionIOLimelight("limelight 1", drive.getRawGyroRotationSupplier()),
                new VisionIOLimelight("limelight 2", drive.getRawGyroRotationSupplier()),
                new VisionIOLimelight("limelight 3", drive.getRawGyroRotationSupplier()),
                new VisionIOPhotonVision("photon", new Transform3d()));

        climberArm = new ClimberArm(new ClimberArmIOTalonFX(14, 5));

        csFlywheel =
            new CoralScorerFlywheel(
                new FlywheelIOTalonFX(),
                new CoralSensorIO() {},
                CoralState.DEFAULT,
                AlgaeState.DEFAULT);
        led = new LED(new LED_IOCANdle(0, "CAN Bus 2"));
        superStructure = new SuperStructure(drive, elevator, csArm, csFlywheel, led);
        climberArm = new ClimberArm(new ClimberArmIOTalonFX(0, 0));
        winch = new Winch(new WinchIOTalonFX(0, 0));

        break;
        // coralIntake = new IntakeFromSource(new CoralScorerFlywheel(), new CoralScorerArm(), new
        // Elevator());
        // objectDetection = new ObjectDetection(new ObjectDetectionConsumer() {}, new
        // ObjectDetectionIO() {});

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        csArm = new CoralScorerArm(new CoralScorerArmIOSim());
        vision =
            new Vision(
                drive.getToPoseEstimatorConsumer(),
                new VisionIOLimelight("limelight 1", drive.getRawGyroRotationSupplier()),
                new VisionIOLimelight("limelight 2", drive.getRawGyroRotationSupplier()),
                new VisionIOLimelight("limelight 3", drive.getRawGyroRotationSupplier()),
                new VisionIOPhotonVision("photon", new Transform3d()));
        elevator = new Elevator(new ElevatorIOSim());
        csFlywheel =
            new CoralScorerFlywheel(
                new FlywheelIOSim(),
                new CoralSensorIO() {},
                CoralState.DEFAULT,
                AlgaeState.DEFAULT);
        led = new LED(new LED_IOSim());

        // objectDetection = new ObjectDetection(new ObjectDetectionConsumer() {}, new
        // ObjectDetectionIO() {});
        superStructure = new SuperStructure(drive, elevator, csArm, csFlywheel, led);
        SmartDashboard.putString("Target State", "" + superStructure.getWantedState());
    SmartDashboard.putString("Current State", "" + superStructure.getCurrentState());

        climberArm = new ClimberArm(new ClimberArmIOSim());
        winch = new Winch(new WinchIOSim());

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

        csArm = new CoralScorerArm(new CoralScorerArmIOSim());
        // vision =
        //     new Vision(
        //         drive.getToPoseEstimatorConsumer(),
        //         new VisionIOLimelight("limelight 1", drive.getRawGyroRotationSupplier()),
        //         new VisionIOLimelight("limelight 2", drive.getRawGyroRotationSupplier()),
        //         new VisionIOLimelight("limelight 3", drive.getRawGyroRotationSupplier()),
        //         new VisionIOPhotonVision("photon", new Transform3d()));
        elevator = new Elevator(new ElevatorIO() {});
        csFlywheel =
            new CoralScorerFlywheel(
                new FlywheelIO() {},
                new CoralSensorIO() {},
                CoralState.DEFAULT,
                AlgaeState.DEFAULT);
        led = new LED(new LED_IO() {});

        winch = new Winch(new WinchIO() {});

        superStructure = new SuperStructure(drive, elevator, csArm, csFlywheel, led);
        break;
    }

    superStructureCommands =
        new SelectCommand<>(
            Map.ofEntries(
                Map.entry(SuperStructureState.STOW, superStructure.getSuperStructureCommand()),
                Map.entry(SuperStructureState.L1, superStructure.getSuperStructureCommand()),
                Map.entry(SuperStructureState.L2, superStructure.getSuperStructureCommand()),
                Map.entry(SuperStructureState.L3, superStructure.getSuperStructureCommand()),
                Map.entry(SuperStructureState.L4, superStructure.getSuperStructureCommand()),
                Map.entry(SuperStructureState.SOURCE, superStructure.getSuperStructureCommand()),
                Map.entry(
                    SuperStructureState.SCORING_CORAL, superStructure.getSuperStructureCommand()),
                Map.entry(
                    SuperStructureState.CLIMB_STAGE_ONE, superStructure.getSuperStructureCommand()),
                Map.entry(
                    SuperStructureState.CLIMB_STAGE_TWO,
                    superStructure.getSuperStructureCommand())),
            () -> superStructure.getWantedState());
    // Set up auto routines
    // NamedCommands.registerCommand("AlignToReefAuto", new AlignToReefAuto(drive, led));

    NamedCommands.registerCommand(
        "L1",
        new SequentialCommandGroup(
            new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L1)),
            new WaitUntilCommand(() -> superStructure.atGoals())));
    NamedCommands.registerCommand(
        "L2",
        new SequentialCommandGroup(
            new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L2)),
            new WaitUntilCommand(() -> superStructure.atGoals())));
    NamedCommands.registerCommand(
        "L3",
        new SequentialCommandGroup(
            new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L3)),
            new WaitUntilCommand(() -> superStructure.atGoals())));
    NamedCommands.registerCommand(
        "L4",
        new SequentialCommandGroup(
            new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L4)),
            new WaitUntilCommand(() -> superStructure.atGoals())));
    NamedCommands.registerCommand(
        "STOW ", new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.STOW)));
    NamedCommands.registerCommand(
        "SCORE",
        new SequentialCommandGroup(
            new InstantCommand(
                () -> superStructure.setWantedState(SuperStructureState.SCORING_CORAL)),
            new WaitUntilCommand(() -> superStructure.atGoals())));
    NamedCommands.registerCommand(
        "INTAKE",
        new SequentialCommandGroup(
            new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.SOURCE)),
            new WaitUntilCommand(() -> superStructure.atGoals())));

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

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

    // angles in none and retract aren't set, CHANGE THEM!!
    //  climbCommands =
    //      new SelectCommand<>(
    //          Map.ofEntries(
    //              Map.entry(
    //                  CLIMB_STATES.EXTEND,
    //                  algaeArm
    //                      .setArmTarget(160, 5)
    //                      .andThen(climbStateMachine::advanceTargetState, algaeArm)),
    //              Map.entry(
    //                  CLIMB_STATES.RETRACT,
    //                  algaeArm
    //                      .setArmTarget(60, 5)
    //                      .andThen(climbStateMachine::advanceTargetState, algaeArm)),
    //              Map.entry(
    //                  CLIMB_STATES.NONE,
    //                  algaeArm
    //                      .setArmTarget(30, 5)
    //                      .andThen(climbStateMachine::advanceTargetState, algaeArm))),
    //          this::climbSelect);
    // Set up auto routines

    // Set up SysId routines

    NamedCommands.registerCommand(
        "ReleaseClawL1",
        new ReleaseClawParallel(FieldConstants.ReefHeight.L1, elevator, csArm, csFlywheel));
    NamedCommands.registerCommand(
        "ReleaseClawL2",
        new ReleaseClawParallel(FieldConstants.ReefHeight.L2, elevator, csArm, csFlywheel));
    NamedCommands.registerCommand(
        "ReleaseClawL3",
        new ReleaseClawParallel(FieldConstants.ReefHeight.L3, elevator, csArm, csFlywheel));
    NamedCommands.registerCommand(
        "ReleaseClawL4",
        new ReleaseClawParallel(FieldConstants.ReefHeight.L4, elevator, csArm, csFlywheel));

    //  NamedCommands.registerCommand("AlignToReefAuto", new AlignToReefAuto(drive, led));
    //  NamedCommands.registerCommand("AutoAlignToSource", new AutoAlignToSource(drive, led));
    NamedCommands.registerCommand(
        "IntakeFromSource",
        new IntakeFromSourceParallel(csFlywheel, csArm, elevator)
            .until(
                () ->
                    csFlywheel.seesCoral() == CoralState.SENSOR
                        || csFlywheel.seesCoral() == CoralState.CURRENT)
            .withTimeout(5));
    NamedCommands.registerCommand(
        "IntakingAlgae",
        new IntakingAlgaeParallel(elevator, csArm, csFlywheel)
            .until(() -> csFlywheel.seesAlgae() == AlgaeState.CURRENT)
            .withTimeout(5));
    NamedCommands.registerCommand("Stow", new Stow(elevator, csArm));

    // NamedCommands.registerCommand(
    // "AlgaeIntoProcessor", new AlgaeIntoProcessor(elevator, csArm, csFlywheel));
    // NamedCommands.registerCommand("ReadyForAlgaeScore", new ReadyForAlgaeScore(elevator, csArm));

    NamedCommands.registerCommand("ReleaseAlgae", new ReleaseAlgae(csFlywheel));

    //  NamedCommands.registerCommand("AutoPickupCoral", new AutoPickupCoral(null, drive, led));

    autos = new SendableChooser<>();

    // autos.addOption("AutoTest",
    // AutoBuilder.buildAuto("Bottom-R5a(L4)-S3c-R6a(L4)-F2-R6b(L4)-S2c"));
    // autos.addOption("AutoTestTwo",
    // AutoBuilder.buildAuto("Bottom-R5a(L4)-F2-R6b(L4)-F2-R6a(L4)"));
    autos.addOption(
        "AutoSourceBottom", AutoBuilder.buildAuto("Bottom-R5a(L4)-F2-R6b(L4)-F2-R6a(L4)"));
    autos.addOption(
        "AutoSourceMiddle", AutoBuilder.buildAuto("Middle-R5a(L4)-F2-R6b(L4)-F2-R6a(L4)"));
    autos.addOption("AutoSourceTop", AutoBuilder.buildAuto("Top-R5a(L4)-F2-R6b(L4)-F2-R6a(L4)"));
    autos.addOption("AutoTestTop", AutoBuilder.buildAuto("Top-R3b(L4)-F1-R2a(L4)-F1-R2b(L4)"));
    autos.addOption(
        "AutoTestMiddle", AutoBuilder.buildAuto("Middle-R3b(L4)-F1-R2a(L4)-F1-R2b(L4)"));
    autos.addOption(
        "AutoTestBottom", AutoBuilder.buildAuto("Bottom-R3b(L4)-F1-R2a(L4)-F1-R2b(L4)"));

    //     autoChooser.addOption(
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
    // autoChooser.addDefaultOption("square", AutoBuilder.buildAuto("Square"));
    // autoChooser.addOption("toReefTest", AutoBuilder.buildAuto("toReefTest"));

    //   autoChooser = new LoggedDashboardChooser<>("Auto Choices", autos);
    //   // Configure the button bindings
    //   // configureButtonBindings();
    //   stateTrigger = new Trigger(() -> superStructure.changedStated());
    // elevatorBrakeTrigger = new Trigger(() -> RobotController.getUserButton());
    // slowModeTrigger = new Trigger(() -> superStructure.elevatorExtended());
    // speedModeTrigger = new Trigger(() -> superStructure.elevatorExtended());
    // configureButtonBindings();
    // test();
    configureButtonBindings();
  }

  /*
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

    driveController.a().onTrue(climberArm.setArmTarget(90, 1));
    driveController.b().onTrue(climberArm.setArmTarget(0, 1));
    driveController.y().onTrue(climberArm.zero());
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
    // slowModeTrigger.onTrue(new InstantCommand(() -> drive.enableSlowMode(true)));
    // slowModeTrigger.onFalse(new InstantCommand(() -> drive.enableSlowMode(false)));

    driverControls();
    manipControls();
  }

  private void driverControls() {
    driveController
        .b()
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
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX(),
            () -> driveController.leftBumper().getAsBoolean(),
            () -> driveController.leftTrigger().getAsBoolean(),
            () -> driveController.rightTrigger().getAsBoolean()));

    driveController
        .leftBumper()
        .onFalse(
            new ConditionalCommand(
                new ApproachReefPerpendicular(drive, superStructure).withTimeout(2),
                new InstantCommand(),
                () ->
                    (!drive.isNearReef() && drive.isAtReefSide())
                        && superStructure.isTargetAReefState()));

    driveController
        .rightBumper()
        .onTrue(
            new WaitUntilCommand(() -> superStructure.atGoals())
                .andThen(
                    new ReinitializingCommand(
                        () -> superStructure.getSuperStructureCommand(),
                        elevator,
                        csArm,
                        csFlywheel,
                        drive,
                        led))
                .andThen(new InstantCommand(() -> superStructure.advanceWantedState())));

    driveController
        .a()
        .onTrue(
            new InstantCommand(
                () -> superStructure.setWantedState(SuperStructureState.CLIMB_STAGE_ONE)));
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
                        csArm,
                        csFlywheel,
                        drive,
                        led)));
  }

  private void testControls() {
    // speedTrigger.onTrue(
    //     new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.STOW))
    //         .andThen(superStructure.getSuperStructureCommand()));
    // keyboard.getCButton().whileTrue(new AlignToCage(drive));
    // keyboard
    //     .getXButton()
    //     .onTrue(new IntakingAlgaeParallel(elevator, csArm, csFlywheel, ReefHeight.L2));
    // keyboard
    //     .getZButton()
    //     .onTrue(new IntakingAlgaeParallel(elevator, csArm, csFlywheel, ReefHeight.L1));
    // stateTrigger.onTrue(new ReinitializingCommand(
    //     () -> superStructure.getSuperStructureCommand(),
    //     elevator, csArm, csFlywheel, led
    //     ));
    // slowModeTrigger.onTrue(new InstantCommand(() -> drive.enableSlowMode(true)));
    // slowModeTrigger.onFalse(new InstantCommand(() -> drive.enableSlowMode(false)));

    // Default command, normal field-relative drive
    // drive.setDefaultCommand(
    //     DriveCommands.joystickDrive(
    //         drive,
    //         () -> -driveController.getLeftY(),
    //         () -> -driveController.getLeftX(),
    //         () -> -driveController.getRightX(),
    //         // () -> driveController.a().getAsBoolean(),
    //         () -> driveController.leftBumper().getAsBoolean(),
    //         () -> driveController.leftTrigger().getAsBoolean(),
    //         () -> driveController.rightTrigger().getAsBoolean(),
    //         () -> driveController.rightBumper().getAsBoolean(),
    //         () -> driveController.b().getAsBoolean(),
    //         () -> driveController.x().getAsBoolean()));

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            superStructure,
            () -> -driveController.getLeftX(),
            () -> -driveController.getLeftY(),
            () -> -driveController.getRightX(),
            () -> driveController.leftBumper().getAsBoolean(),
            () -> driveController.leftTrigger().getAsBoolean(),
            () -> driveController.rightTrigger().getAsBoolean()));

    // driveController.x().onTrue(new Stow(elevator, csArm));

    // driveController
    //     .a()
    //     .onTrue(
    //         new ConditionalCommand(
    //             new ApproachReefPerpendicular(drive, superStructure).withTimeout(2),
    //             new InstantCommand(),
    //             () -> (!drive.isNearReef() && drive.isAtReefSide() &&
    // drive.isAtReefRotation())));

    // driveController.x().onTrue(new Stow(elevator, csArm));
    //   elevatorBrakeTrigger.onTrue(new InstantCommand(() -> elevator.breakMode(true), elevator));
    //  elevatorBrakeTrigger.onFalse(new InstantCommand(() -> elevator.breakMode(false)));

    //
    // driveController.y().onFalse(new InstantCommand(() -> csFlywheel.stop(), csFlywheel));
    // driveController
    //     .y()
    //     .onTrue(new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L4)));
    // driveController
    //     .povDown()
    //     .onTrue(new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L2)));
    // driveController
    //     .povUp()
    //     .onTrue(
    //         new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.STOW))
    //             .andThen(superStructure.getSuperStructureCommand()));
    // driveController
    //     .a()
    //     .onFalse(
    //         new InstantCommand(
    //             () -> superStructure.setWantedState(SuperStructureState.SCORING_CORAL)));
    // stateTrigger.onTrue(superStructure.getSuperStructureCommand());
    // driveController.a().onFalse(new SetClawLevel(ElevatorState.STOW, elevator, csArm));
    // driveController
    //     .rightBumper()
    //     .onTrue(new InstantCommand(() ->
    // superStructure.setWantedState(SuperStructureState.SOURCE)));

    // driveController
    //     .rightBumper()
    //     .onFalse(new InstantCommand(() ->
    // superStructure.setWantedState(SuperStructureState.STOW)));

    // // driveController
    // //   .a()
    // // .onTrue(new InstantCommand(() -> Super.setWantedState(SuperStructureState.L1)));

    // driveController.a().onTrue(new AlignToReefAuto(drive, led));
    // driveController
    //     .x()
    //     .onTrue(new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L3)));
    // //  driveController
    // //    .a()
    // //   .onFalse(new InstantCommand(() -> Super.setWantedState(SuperStructureState.STOW)));

    // driveController.y().onTrue(new AlgaeIntoProcesser(elevator, csArm, csFlywheel));
    // driveController.y().onFalse(new Stow(elevator, csArm));

    // why is this like this?
    // driveController.leftBumper().onTrue(new InstantCommand(() -> drive.setNearestReefSide()));
    // driveController.rightBumper().onTrue(new SetClawLevel(ElevatorState.SOURCE, elevator,
    // csArm));
    // driveController.rightBumper().onFalse(new SetClawLevel(ElevatorState.STOW, elevator, csArm));
    // driveController.leftBumper().whileTrue(new AutoAlignToSource(drive, led));
    // driveController.rightBumper().whileTrue(new AlignToReefAuto(drive, led));
    // driveController.rightTrigger().whileTrue(new AlignToProcessor(drive, led));

    // // Lock to 0° when A button is held
    // controller
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> new Rotation2d()));

    // // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // controller.y().onTrue(csArm.setArmTarget(30, 0));

    // // Reset gyro to 0° when B button is pressed
    // controller
    //     .b()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     drive.setPose(
    //                         new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
    //                 drive)
    //             .ignoringDisable(true));

    // All button bindings (not related to drive)

    // Drive Controller - Align Commands go in Drive
    driveController.b().onTrue((new ReleaseAlgae(csFlywheel)));
    driveController.b().onFalse(new InstantCommand(() -> csFlywheel.runVolts(0)));

    // Manip Controller
    manipController.rightTrigger().onTrue(new Stow(elevator, csArm));

    manipController
        .leftBumper()
        .onTrue(
            new IntakeFromSourceParallel(csFlywheel, csArm, elevator)
                .until(
                    () ->
                        csFlywheel.seesCoral() == CoralState.SENSOR
                            || csFlywheel.seesCoral() == CoralState.CURRENT)
                .withTimeout(5));
    manipController
        .leftBumper()
        .onFalse(
            new ParallelCommandGroup(
                new Stow(elevator, csArm), new InstantCommand(() -> csFlywheel.runVolts(0))));

    manipController
        .rightBumper()
        .onTrue(
            new IntakingAlgaeParallel(elevator, csArm, csFlywheel)
                .until(() -> csFlywheel.seesAlgae() == AlgaeState.CURRENT)
                .withTimeout(5));
    manipController
        .rightBumper()
        .onFalse(
            new ParallelCommandGroup(
                new Stow(elevator, csArm), new InstantCommand(() -> csFlywheel.runVolts(0))));

    manipController
        .a()
        .onTrue(new ReleaseClawParallel(FieldConstants.ReefHeight.L1, elevator, csArm, csFlywheel));
    manipController
        .a()
        .onFalse(
            new ParallelCommandGroup(
                new Stow(elevator, csArm), new InstantCommand(() -> csFlywheel.runVolts(0))));

    manipController
        .b()
        .onTrue(new ReleaseClawParallel(FieldConstants.ReefHeight.L2, elevator, csArm, csFlywheel));
    manipController
        .b()
        .onFalse(
            new ParallelCommandGroup(
                new Stow(elevator, csArm), new InstantCommand(() -> csFlywheel.runVolts(0))));

    manipController
        .x()
        .onTrue(new ReleaseClawParallel(FieldConstants.ReefHeight.L3, elevator, csArm, csFlywheel));
    manipController
        .x()
        .onFalse(
            new ParallelCommandGroup(
                new Stow(elevator, csArm), new InstantCommand(() -> csFlywheel.runVolts(0))));

    manipController
        .y()
        .onTrue(new ReleaseClawParallel(FieldConstants.ReefHeight.L4, elevator, csArm, csFlywheel));
    manipController
        .y()
        .onFalse(
            new ParallelCommandGroup(
                new Stow(elevator, csArm), new InstantCommand(() -> csFlywheel.runVolts(0))));

    // controller.y().whileTrue(elevator.setElevatorTarget(1.83, 1));
    // controller.y().whileFalse(elevator.setElevatorTarget(1, 1));

    // controller.x().whileTrue(csArm.setArmTarget(90, 1));
    // controller.x().whileFalse(csArm.setArmTarget(-90, 1));

    // controller.b().whileTrue(algaeArm.setArmTarget(70, 2));
    // controller.b().whileFalse(algaeArm.setArmTarget(20, 2));

    // manipController.rightBumper().onTrue(new IntakingAlgaeParallel(elevator, csArm, csFlywheel));
    // manipController
    //     .rightBumper()
    //     .onFalse(
    //         new ParallelCommandGroup(
    //             csArm.setArmTarget(60, 4),
    //             elevator.setElevatorTarget(0.2, 0.05),
    //             new InstantCommand(() -> csFlywheel.runVolts(12))));

    // manipController.rightTrigger().onTrue(new Stow(csArm, elevator));
    // // driveController.a().whileTrue(new ReleaseClawParallel(scoringLevel, elevator, csArm,
    // // csFlywheel));
    // driveController.rightBumper().onTrue(new AlignToReefAuto(drive, led));

    // driveController.leftBumper().onTrue(new AutoAlignToSource(drive, led));

    // // driveController.rightTrigger().onTrue(new ReleaseAlgae(csFlywheel));

    // // manipController.a().onTrue(new InstantCommand(() ->
    // // elevator.setElevatorTarget(FieldConstants.ReefHeight.L1.height, 1)));
    // // manipController.b().onTrue(new InstantCommand(() ->
    // // elevator.setElevatorTarget(FieldConstants.ReefHeight.L2.height, 1)));
    // // manipController.x().onTrue(new InstantCommand(() ->
    // // elevator.setElevatorTarget(FieldConstants.ReefHeight.L3.height, 1)));
    // // manipController.y().onTrue(new InstantCommand(() ->
    // // elevator.setElevatorTarget(FieldConstants.ReefHeight.L4.height, 1)));

    // manipController
    //     .a()
    //     .onTrue(new ReleaseClawParallel(FieldConstants.ReefHeight.L1, elevator, csArm,
    // csFlywheel));
    // manipController
    //     .b()
    //     .onTrue(new ReleaseClawParallel(FieldConstants.ReefHeight.L2, elevator, csArm,
    // csFlywheel));

    // // driveController
    // //     .a()
    // //     .onTrue(new ReleaseClawParallel(FieldConstants.ReefHeight.L1, elevator, csArm,
    // csFlywheel));
    // driveController
    //     .b()
    //     .onTrue(new ReleaseClawParallel(FieldConstants.ReefHeight.L2, elevator, csArm,
    // csFlywheel));
    // driveController
    //     .x()
    //     .onTrue(new ReleaseClawParallel(FieldConstants.ReefHeight.L3, elevator, csArm,
    // csFlywheel));
    // driveController
    //     .y()
    //     .onTrue(new ReleaseClawParallel(FieldConstants.ReefHeight.L4, elevator, csArm,
    // csFlywheel));
    // driveController.leftBumper().onTrue(new AutoAlignToSource(drive, led));

    // keyboard
    //    .getCButton()
    //    .onTrue(new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L1)));
    // keyboard
    //    .getVButton()
    //    .onTrue(new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L2)));

    manipController
        .leftBumper()
        .onTrue(
            new ReinitializingCommand(
                () -> superStructure.getSuperStructureCommand(), elevator, csArm, csFlywheel, led));
    // manipController.leftBumper().whileTrue(new AutoAlignToSource(drive, led));
    // manipController.leftBumper().onTrue(new IntakeFromSourceParallel(csFlywheel, csArm,
    // elevator));
    // manipController.leftBumper().onFalse(new Stow(elevator, csArm));
    manipController.rightBumper().onTrue(new IntakingAlgaeParallel(elevator, csArm, csFlywheel));
    manipController.rightBumper().onFalse(new Stow(elevator, csArm));
    // driveController.rightTrigger().onTrue(new ReleaseAlgae(csFlywheel));

    // controller.y().onTrue(climbCommands);
    // controller
    //     .a()
    //     .onTrue(
    //         new IntakeFromSourceParallel(csFlywheel, csArm, elevator)
    //             .until(
    //                 () ->
    //                     csFlywheel.seesCoral() == CoralState.SENSOR
    //                         || csFlywheel.seesCoral() == CoralState.CURRENT)
    //             .withTimeout(5));
    // controller
    //     .a()
    //     .onFalse(
    //         new ParallelCommandGroup(
    //             csArm.setArmTarget(60, 4),
    //             elevator.setElevatorTarget(0.2, 0.05),
    //             new InstantCommand(() -> csFlywheel.runVolts(0))));

    // controller
    //     .a()
    //     .onTrue(
    //         new IntakingAlgaeParallel(elevator, csArm, csFlywheel)
    //             .until(() -> csFlywheel.seesAlgae() == AlgaeState.CURRENT)
    //             .withTimeout(5));
    // controller
    //     .a()
    //     .onFalse(
    //         new ParallelCommandGroup(
    //             csArm.setArmTarget(60, 4),
    //             elevator.setElevatorTarget(0.2, 0.05),
    //             new InstantCommand(() -> csFlywheel.runVolts(0))));
    // controller
    //     .leftBumper()
    //     .onFalse(
    //         new ParallelCommandGroup(
    //             csArm.setArmTarget(60, 4),
    //             elevator.setElevatorTarget(0.2, 0.05),
    //             new InstantCommand(() -> csFlywheel.runVolts(0))));
    // controller
    //     .leftBumper()
    //     .onTrue(new ReleaseClawParallel(FieldConstants.ReefHeight.L4, elevator, csArm,
    // csFlywheel));
    // manipController
    //     .a()
    //     .onTrue(new ReleaseClawParallel(FieldConstants.ReefHeight.L1, elevator, csArm,
    // csFlywheel));
    // manipController
    //     .b()
    //     .onTrue(new ReleaseClawParallel(FieldConstants.ReefHeight.L2, elevator, csArm,
    // csFlywheel));

    // driveController
    //     .a()
    //     .onTrue(new ReleaseClawParallel(FieldConstants.ReefHeight.L1, elevator, csArm,
    // csFlywheel));
    // driveController
    //     .b()
    //     .onTrue(new ReleaseClawParallel(FieldConstants.ReefHeight.L2, elevator, csArm,
    // csFlywheel));
    // driveController
    //     .x()
    //     .onTrue(new ReleaseClawParallel(FieldConstants.ReefHeight.L3, elevator, csArm,
    // csFlywheel));
    // driveController
    //     .y()
    //     .onTrue(new ReleaseClawParallel(FieldConstants.ReefHeight.L4, elevator, csArm,
    // csFlywheel));

    // controller.y().onTrue(climbCommands);

    // driveController.leftTrigger().onTrue(new IntakingAlgaeParallel(elevator, csArm, csFlywheel));
    // controller
    //     .leftTrigger()
    //     .onFalse(
    //         new ParallelCommandGroup(
    //             csArm.setArmTarget(60, 4),
    //             elevator.setElevatorTarget(0.2, 0.05),
    //             new InstantCommand(() -> csFlywheel.runVolts(0))));
    // controller
    //     .leftBumper()
    //     .onTrue(new ReleaseClawParallel(FieldConstants.ReefHeight.L4, elevator, csArm,
    // csFlywheel));

  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous.
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public CoralScorerArm getScoralArm() {
    return csArm;
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
}
