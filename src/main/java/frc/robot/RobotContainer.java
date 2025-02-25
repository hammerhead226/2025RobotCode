package frc.robot;

import static frc.robot.constants.RobotMap.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ApproachReefPerpendicular;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ReinitializingCommand;
import frc.robot.constants.RobotMap;
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
import frc.robot.subsystems.commoniolayers.FlywheelIO;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LED_IO;
import frc.robot.subsystems.led.LED_IOCANdle;
import frc.robot.subsystems.led.LED_IOSim;
import frc.robot.subsystems.scoral.ScoralArm;
import frc.robot.subsystems.scoral.ScoralArmIOSim;
import frc.robot.subsystems.scoral.ScoralArmIOTalonFX;
import frc.robot.subsystems.scoral.ScoralRollers;
import frc.robot.subsystems.scoral.ScoralRollersIOSim;
import frc.robot.subsystems.scoral.ScoralRollersIOTalonFX;
import frc.robot.subsystems.scoral.ScoralSensorCANRangeIO;
import frc.robot.subsystems.scoral.ScoralSensorIO;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
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
  // private final Joystick joystikc = new Joystick(0);
  // private final JoystickButton btn = new JoystickButton(joystikc, 4);
  // private final KeyboardInputs keyboard = new KeyboardInputs(0);

  private ScoralArm scoralArm;
  // private final CoralScorerFlywheel coralIntake;

  public static Elevator elevator;
  public ClimberArm climberArm;
  private Vision vision;
  SuperStructure superStructure;
  private final Winch winch;

  // public final Trigger elevatorBrakeTrigger;
  // private final Trigger stateTrigger;
  // private final Trigger slowModeTrigger;

  private ScoralRollers scoralRollers;

  public final Trigger elevatorBrakeTrigger;
  // private final Trigger stateTrigger;
  private Trigger slowModeTrigger;
  private Trigger reefAlignTrigger;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final SendableChooser<Command> autos;
  private DigitalInput brakeSwitch;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (SimConstants.currentMode) {
      case REAL:
        brakeSwitch = new DigitalInput(2);
        elevatorBrakeTrigger = new Trigger(() -> brakeSwitch.get());
        // // Real robot, instantiate hardware IO implementations

        elevator =
            new Elevator(
                new ElevatorIOTalonFX(
                    RobotMap.ElevatorIDs.leftElevatorID,
                    RobotMap.ElevatorIDs.rightElevatorID,
                    RobotMap.ElevatorIDs.elevatorCANrangeID));
        // winch = new Winch(new WinchIOTalonFX(12, 13));
        winch = new Winch(new WinchIO() {});
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        scoralArm =
            new ScoralArm(
                new ScoralArmIOTalonFX(
                    RobotMap.CoralScorerArmIDs.coralScorerRotationID,
                    RobotMap.CoralScorerArmIDs.coralScorerRotationCANcoderID));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight("limelight-reef", drive.getRawGyroRotationSupplier())
                // new VisionIOLimelight("limelight 2", drive.getRawGyroRotationSupplier()),
                // new VisionIOLimelight("limelight 3", drive.getRawGyroRotationSupplier()),
                // new VisionIOPhotonVision("photon", new Transform3d())
                );

        climberArm = new ClimberArm(new ClimberArmIOTalonFX(14, 16));
        // climberArm = new ClimberArm(new ClimberArmIO() {});

        // scoralRollers =
        // new ScoralRollers(
        // new ScoralRollersIOSim(),
        // new ScoralSensorIO() {},
        // CoralState.DEFAULT,
        // AlgaeState.DEFAULT);
        scoralRollers =
            new ScoralRollers(
                new ScoralRollersIOTalonFX(RobotMap.CoralScorerArmIDs.coralScorerFlywheelID),
                new ScoralSensorCANRangeIO(RobotMap.CoralScorerArmIDs.coralScorerCANrangeID),
                CoralState.DEFAULT,
                AlgaeState.DEFAULT);
        led = new LED(new LED_IOCANdle(0, "CAN Bus 2"));
        superStructure =
            new SuperStructure(drive, elevator, scoralArm, scoralRollers, led, climberArm);

        break;
      case SIM:
        elevatorBrakeTrigger = new Trigger(() -> true);
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
        // Transform3d bruh = new Transform3d(new Transform2d());
        Rotation3d bruh = new Rotation3d();
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    "camera 1 sim", new Transform3d(0, 0, 0, bruh), drive::getPose),
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

        climberArm = new ClimberArm(new ClimberArmIOSim());

        superStructure =
            new SuperStructure(drive, elevator, scoralArm, scoralRollers, led, climberArm);
        break;

      default:
        elevatorBrakeTrigger = new Trigger(() -> true);
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
        // new Vision(
        // drive.getToPoseEstimatorConsumer(),
        // new VisionIOLimelight("limelight 1", drive.getRawGyroRotationSupplier()),
        // new VisionIOLimelight("limelight 2", drive.getRawGyroRotationSupplier()),
        // new VisionIOLimelight("limelight 3", drive.getRawGyroRotationSupplier()),
        // new VisionIOPhotonVision("photon", new Transform3d()));
        elevator = new Elevator(new ElevatorIO() {});
        scoralRollers =
            new ScoralRollers(
                new FlywheelIO() {},
                new ScoralSensorIO() {},
                CoralState.DEFAULT,
                AlgaeState.DEFAULT);
        led = new LED(new LED_IO() {});

        winch = new Winch(new WinchIO() {});

        superStructure =
            new SuperStructure(drive, elevator, scoralArm, scoralRollers, led, climberArm);
        break;
    }

    // Set up auto routines
    // NamedCommands.registerCommand("AlignToReefAuto", new AlignToReefAuto(drive,
    // led));

    // NamedCommands.registerCommand("L1", new SequentialCommandGroup(
    // new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L1)), new
    // WaitUntilCommand(() -> superStructure.atGoals())));

    // NamedCommands.registerCommand("L2", new SequentialCommandGroup(
    // new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L2)), new
    // WaitUntilCommand(() -> superStructure.atGoals())));

    // NamedCommands.registerCommand("L3", new SequentialCommandGroup( new InstantCommand(() ->
    // superStructure.setWantedState(SuperStructureState.L3)), new WaitUntilCommand(() ->
    // superStructure.atGoals())));

    // NamedCommands.registerCommand(
    // "L4",
    // new SequentialCommandGroup(
    // new InstantCommand(() ->
    // superStructure.setWantedState(SuperStructureState.L4)),
    // new WaitUntilCommand(() -> superStructure.atGoals())));

    // NamedCommands.registerCommand(
    // "INTAKE",
    // new SequentialCommandGroup(
    // new InstantCommand(() ->
    // superStructure.setWantedState(SuperStructureState.SOURCE)),
    // new WaitUntilCommand(() -> superStructure.atGoals())));

    // autoChooser = new LoggedDashboardChooser<>("Auto Choices",
    // AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    // autoChooser.addOption(
    // "Drive Wheel Radius Characterization",
    // DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    // "Drive Simple FF Characterization",
    // DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    // "Drive SysId (Quasistatic Forward)",
    // drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    // "Drive SysId (Quasistatic Reverse)",
    // drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    // "Drive SysId (Dynamic Forward)",
    // drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    // "Drive SysId (Dynamic Reverse)",
    // drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    // "Top R3a",
    // AutoBuilder.buildAuto("R3a(L3)-S1c-R2a(L3)-S2c-R1b(L3)-S3c-R6a(L3)"));
    // autoChooser.addDefaultOption("square", AutoBuilder.buildAuto("Square"));
    // autoChooser.addDefaultOption("1.1 auto", AutoBuilder.buildAuto("1.1.auto"));

    // Set up auto routines

    // Set up SysId routines

    NamedCommands.registerCommand(
        "L1",
        new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L1))
            .andThen(new WaitUntilCommand(() -> superStructure.atGoals()))
            .andThen(superStructure.getSuperStructureCommand()));
    NamedCommands.registerCommand(
        "L2",
        new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L2))
            .andThen(new WaitUntilCommand(() -> superStructure.atGoals()))
            .andThen(superStructure.getSuperStructureCommand()));
    NamedCommands.registerCommand(
        "L3",
        new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L3))
            .andThen(new WaitUntilCommand(() -> superStructure.atGoals()))
            .andThen(superStructure.getSuperStructureCommand()));
    NamedCommands.registerCommand(
        "L4",
        new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L4))
            .andThen(new WaitUntilCommand(() -> superStructure.atGoals()))
            .andThen(superStructure.getSuperStructureCommand()));

    NamedCommands.registerCommand(
        "SOURCE_INTAKE",
        new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.SOURCE))
            .andThen(new WaitUntilCommand(() -> superStructure.atGoals()))
            .andThen(superStructure.getSuperStructureCommand()));
    NamedCommands.registerCommand(
        "ALGAE_INTAKE",
        new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.INTAKE_ALGAE))
            .andThen(new WaitUntilCommand(() -> superStructure.atGoals()))
            .andThen(superStructure.getSuperStructureCommand()));
    NamedCommands.registerCommand(
        "STOW",
        new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.STOW))
            .andThen(new WaitUntilCommand(() -> superStructure.atGoals()))
            .andThen(superStructure.getSuperStructureCommand()));
    NamedCommands.registerCommand(
        "SCORE_CORAL",
        new SequentialCommandGroup(
            new InstantCommand(
                () -> superStructure.setWantedState(SuperStructureState.SCORING_CORAL)),
            new WaitUntilCommand(() -> superStructure.atGoals()),
            superStructure.getSuperStructureCommand()));
    // NamedCommands.registerCommand("Stow", new Stow(elevator, csArm));

    autos = new SendableChooser<>();

    // autos.addOption("AutoTest",
    // AutoBuilder.buildAuto("Bottom-R5a(L4)-S3c-R6a(L4)-F2-R6b(L4)-S2c");
    // autos.addOption("AutoTestTwo",
    // AutoBuilder.buildAuto("Bottom-R5a(L4)-F2-R6b(L4)-F2-R6a(L4)"));
    // autos.addOption(
    // "AutoSourceBottom",
    // AutoBuilder.buildAuto("Bottom-R5a(L4)-F2-R6b(L4)-F2-R6a(L4)"));
    // autos.addOption(
    // "AutoSourceMiddle",
    // AutoBuilder.buildAuto("Middle-R5a(L4)-F2-R6b(L4)-F2-R6a(L4)"));
    // autos.addOption("AutoSourceTop",
    // AutoBuilder.buildAuto("Top-R5a(L4)-F2-R6b(L4)-F2-R6a(L4)"));
    // autos.addOption("AutoTestTop",
    // AutoBuilder.buildAuto("Top-R3b(L4)-F1-R2a(L4)-F1-R2b(L4)"));
    // autos.addOption(
    // "AutoTestMiddle",
    // AutoBuilder.buildAuto("Middle-R3b(L4)-F1-R2a(L4)-F1-R2b(L4)"));
    // autos.addOption(
    // "AutoTestBottom",
    // AutoBuilder.buildAuto("Bottom-R3b(L4)-F1-R2a(L4)-F1-R2b(L4)"));

    autos.addOption("BlueLeft", AutoBuilder.buildAuto("BlueLeft"));
    autos.addOption("BlueLeftPush", AutoBuilder.buildAuto("BlueLeftPush"));
    autos.addOption("BlueRight", AutoBuilder.buildAuto("BlueRight"));
    autos.addOption("BlueMiddleLeft", AutoBuilder.buildAuto("BlueMiddleLeft"));
    autos.addOption("BlueMiddleRight", AutoBuilder.buildAuto("BlueMiddleRight"));

    autos.addOption("BlueLeftL2", AutoBuilder.buildAuto("BlueLeftL2"));
    autos.addOption("BlueLeftPushL2", AutoBuilder.buildAuto("BlueLeftPushL2"));
    autos.addOption("BlueRightL2", AutoBuilder.buildAuto("BlueRightL2"));
    autos.addOption("BlueMiddleLeftL2", AutoBuilder.buildAuto("BlueMiddleLeftL2"));
    autos.addOption("BlueMiddleRightL2", AutoBuilder.buildAuto("BlueMiddleRightL2"));

    // autoChooser.addOption(
    // "Drive Wheel Radius Characterization",
    // DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    // "Drive Simple FF Characterization",
    // DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    // "Drive SysId (Quasistatic Forward)",
    // drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    // "Drive SysId (Quasistatic Reverse)",
    // drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    // "Drive SysId (Dynamic Forward)",
    // drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    // "Drive SysId (Dynamic Reverse)",
    // drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addDefaultOption("square", AutoBuilder.buildAuto("Square"));
    // autoChooser.addOption("toReefTest", AutoBuilder.buildAuto("toReefTest"));

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", autos);
    // Configure the button bindings
    // configureButtonBindings();
    // stateTrigger = new Trigger(() -> superStructure.changedStated());

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
    // driveController.a().onTrue(led.setStateCommand(LED_STATE.BLUE));
    // driveController.y().onTrue(led.setStateCommand(LED_STATE.GREEN));
    // driveController.x().onTrue(led.setStateCommand(LED_STATE.FIRE));
    // driveController.a().onTrue(winch.runVoltsCommmand(1));
    // driveController.a().onFalse(new InstantCommand(() -> winch.stop(), winch));
    // driveController.b().onTrue(climberArm.setArmTarget(20, 1));
    // driveController.b().onTrue(climberArm.setArmTarget(0, 1));

    // driveController.y().onFalse(new InstantCommand(() -> scoralArm.armStop()));
    // driveController.x().onTrue(scoralArm.setArmTarget(20, 1));
    // driveController.x().onFalse(new InstantCommand(() -> scoralArm.armStop()));
    // driveController.b().onTrue(scoralRollers.runVoltsCommmand(2));
    // driveController.b().onFalse(scoralRollers.stopCommand());
    // driveController.a().onTrue(new GoToReefHeight(elevator, scoralArm, 10, 2));
    driveController
        .b()
        .onTrue(new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L2)));
    driveController
        .x()
        .onTrue(new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L3)));
    driveController
        .y()
        .onTrue(new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.L4)));
    driveController
        .a()
        .onTrue(
            new InstantCommand(() -> superStructure.setWantedState(SuperStructureState.SOURCE)));
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

    // driveController
    // .povUp()
    // .onTrue(
    // new InstantCommand(
    // () -> superStructure.setWantedState(SuperStructureState.CLIMB_STAGE_ONE)));
    driveController.povUp().onTrue(climberArm.setArmTarget(0, 2));
    driveController.povLeft().onTrue(climberArm.setArmTarget(90, 2));

    // driveController
    // .y()
    // .onTrue(
    // scoralArm.setArmTarget(
    // SubsystemConstants.CoralScorerConstants.ScoralArmConstants.STOW_SETPOINT_DEG,
    // 1));
    // driveController.x().onTrue(scoralRollers.runVoltsCommmand(5));
    // driveController.x().onFalse(scoralRollers.stopCommand());
    // driveController.b().onTrue(new GoToReefHeight(elevator, scoralArm, 7, 74));
    // // L2
    // driveController.a().onTrue(new IntakingCoral(scoralRollers));
    // driveController.a().onFalse(scoralRollers.stopCommand());
    // driveController.b().onTrue(new InstantCommand(() -> climberArm.armStop(),
    // climberArm));
    // driveController
    // .b()
    // .onTrue(
    // new ParallelCommandGroup(
    // winch.runVoltsCommmand(2).until(() -> climberArm.getArmPositionDegs() ==
    // 130),
    // climberArm.setArmTarget(130, 1)));
    // driveController
    // .b()
    // .onFalse(
    // new ParallelCommandGroup(
    // new InstantCommand(() -> winch.stop(), winch),
    // new InstantCommand(() -> climberArm.armStop(), climberArm)));
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

    // elevatorBrakeTrigger.onTrue(
    // new InstantCommand(() -> elevator.setBrake(false)).ignoringDisable(true));
    // elevatorBrakeTrigger.onFalse(
    // new InstantCommand(() -> elevator.setBrake(true)).ignoringDisable(true));
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

  // private void testControls() {
  // slowModeTrigger.onTrue(new InstantCommand(() -> drive.enableSlowMode(true)));
  // slowModeTrigger.onFalse(new InstantCommand(() ->
  // drive.enableSlowMode(false)));

  // drive.setDefaultCommand(
  // DriveCommands.joystickDrive(
  // drive,
  // superStructure,
  // () -> -driveController.getLeftY(),
  // () -> -driveController.getLeftX(),
  // () -> -driveController.getRightX(),
  // () -> driveController.leftBumper().getAsBoolean(),
  // () -> driveController.leftTrigger().getAsBoolean(),
  // () -> driveController.rightTrigger().getAsBoolean()));

  // // Drive Controller - Align Commands go in Drive
  // driveController.b().onTrue((new ReleaseAlgae(csFlywheel)));
  // driveController.b().onFalse(new InstantCommand(() ->
  // csFlywheel.runVolts(0)));

  // // Manip Controller
  // manipController.rightTrigger().onTrue(new Stow(elevator, csArm));

  // manipController
  // .leftBumper()
  // .onTrue(
  // new IntakeFromSourceParallel(csFlywheel, csArm, elevator)
  // .until(
  // () ->
  // csFlywheel.seesCoral() == CoralState.SENSOR
  // || csFlywheel.seesCoral() == CoralState.CURRENT)
  // .withTimeout(5));
  // manipController
  // .leftBumper()
  // .onFalse(
  // new ParallelCommandGroup(
  // new Stow(elevator, csArm), new InstantCommand(() ->
  // csFlywheel.runVolts(0))));

  // manipController
  // .rightBumper()
  // .onTrue(
  // new IntakingAlgaeParallel(elevator, csArm, csFlywheel)
  // .until(() -> csFlywheel.seesAlgae() == AlgaeState.CURRENT)
  // .withTimeout(5));
  // manipController
  // .rightBumper()
  // .onFalse(
  // new ParallelCommandGroup(
  // new Stow(elevator, csArm), new InstantCommand(() ->
  // csFlywheel.runVolts(0))));

  // manipController
  // .a()
  // .onTrue(new ReleaseClawParallel(FieldConstants.ReefHeight.L1, elevator,
  // csArm,
  // csFlywheel));
  // manipController
  // .a()
  // .onFalse(
  // new ParallelCommandGroup(
  // new Stow(elevator, csArm), new InstantCommand(() ->
  // csFlywheel.runVolts(0))));

  // manipController
  // .b()
  // .onTrue(new ReleaseClawParallel(FieldConstants.ReefHeight.L2, elevator,
  // csArm,
  // csFlywheel));
  // manipController
  // .b()
  // .onFalse(
  // new ParallelCommandGroup(
  // new Stow(elevator, csArm), new InstantCommand(() ->
  // csFlywheel.runVolts(0))));

  // manipController
  // .x()
  // .onTrue(new ReleaseClawParallel(FieldConstants.ReefHeight.L3, elevator,
  // csArm,
  // csFlywheel));
  // manipController
  // .x()
  // .onFalse(
  // new ParallelCommandGroup(
  // new Stow(elevator, csArm), new InstantCommand(() ->
  // csFlywheel.runVolts(0))));

  // manipController
  // .y()
  // .onTrue(new ReleaseClawParallel(FieldConstants.ReefHeight.L4, elevator,
  // csArm,
  // csFlywheel));
  // manipController
  // .y()
  // .onFalse(
  // new ParallelCommandGroup(
  // new Stow(elevator, csArm), new InstantCommand(() ->
  // csFlywheel.runVolts(0))));

  // manipController
  // .leftBumper()
  // .onTrue(
  // new ReinitializingCommand(
  // () -> superStructure.getSuperStructureCommand(), elevator, csArm, csFlywheel,
  // led));

  // manipController.rightBumper().onFalse(new Stow(elevator, csArm));
  // }
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

  public ScoralRollers getScoralRollers() {
    return scoralRollers;
  }

  public ClimberArm getClimber() {
    return climberArm;
  }
}
