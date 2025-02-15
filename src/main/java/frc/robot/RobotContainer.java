// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import frc.robot.ClimbStateMachine.java.ClimbStateMachine;
import frc.robot.commands.AlignToReefAuto;
import frc.robot.commands.AutoAlignToSource;
import frc.robot.commands.AutoPickupCoral;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeFromSourceParallel;
import frc.robot.commands.IntakingAlgae;
import frc.robot.commands.IntakingAlgaeParallel;
import frc.robot.commands.ReleaseClawParallel;
import frc.robot.commands.Stow;
import frc.robot.commands.algaeintosource.ReleaseAlgae;
// import frc.robot.commands.algaeintoprocesser.AlgaeIntoProcesser;
import frc.robot.constants.FieldConstants;
// import frc.robot.commands.IntakeFromSource;
import frc.robot.constants.FieldConstants.ReefHeight;
import frc.robot.constants.SimConstants;
import frc.robot.constants.SubsystemConstants.AlgaeState;
import frc.robot.constants.SubsystemConstants.CoralState;
import frc.robot.constants.TunerConstants;
import frc.robot.statemachines.ClimbStateMachine;
import frc.robot.statemachines.ClimbStateMachine.CLIMB_STATES;
import frc.robot.subsystems.coralIntake.flywheels.CoralIntakeSensorIO;
import frc.robot.subsystems.coralscorer.CoralScorerArm;
import frc.robot.subsystems.coralscorer.CoralScorerArmIOSim;
import frc.robot.subsystems.coralscorer.CoralScorerArmIOTalonFX;
import frc.robot.subsystems.coralscorer.CoralScorerFlywheel;
import frc.robot.subsystems.coralscorer.CoralScorerFlywheelIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LED_IO;
import frc.robot.subsystems.led.LED_IOCANdle;
import frc.robot.subsystems.led.LED_IOSim;
import frc.robot.subsystems.newalgaeintake.AlgaeIntakeArm;
import frc.robot.subsystems.newalgaeintake.AlgaeIntakeArmIOSim;
import frc.robot.subsystems.newalgaeintake.AlgaeIntakeArmIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.util.KeyboardInputs;
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
  private final CommandXboxController controller = new CommandXboxController(0);
  private final Joystick joystikc = new Joystick(0);
  private final JoystickButton btn = new JoystickButton(joystikc, 4);
  private final KeyboardInputs keyboard = new KeyboardInputs(0);

  private final CoralScorerArm csArm;
  // private final CoralScorerFlywheel coralIntake;

  private final Elevator elevator;
  private final AlgaeIntakeArm algaeArm;
  private final Vision vision;

  private final ClimbStateMachine climbStateMachine;
  private final Pose2d pose2d;
  // private final ObjectDetection objectDetection;
  // private final ObjectDetectionConsumer odConsumer;
  // private final ObjectDetectionIO odIO;

  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController manipController = new CommandXboxController(1);
  private final CoralScorerFlywheel csFlywheel;

  private Pose2d nearestSide = new Pose2d();

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private CLIMB_STATES climbSelect() {
    return climbStateMachine.getTargetState();
  }

  public ReefHeight getNearestReefSideAndConvertToAlgaeHeight() {
    Translation2d start = FieldConstants.Reef.center;
    Translation2d end = drive.getPose().getTranslation();
    Translation2d v = end.minus(start);
    Rotation2d angle = new Rotation2d(v.getX(), v.getY());

    // https://www.desmos.com/calculator/44dd9koglh

    // negate since branchPositions is CW not CCW
    // +6/12 since branchPositions starts at branch B not the +x axis
    double rawRotations = angle.getRotations();
    double adjustedRotations = -rawRotations + (7.0 / 12.0);

    // % 1 to just get the fractional part of the rotation
    // multiply by 12 before flooring so [0,1) maps to 0,1,2...10,11 evenly
    double fractionalRotation = adjustedRotations % 1;
    if (fractionalRotation < 0) {
      fractionalRotation++;
    }
    int index = (int) Math.floor(fractionalRotation * 12);

    switch (index) {
      case 0, 1, 4, 5, 8, 9:
        return ReefHeight.L1;
      case 2, 3, 6, 7, 10, 11:
        return ReefHeight.L2;
      default:
        return ReefHeight.L1;
    }
  }

  private final Command climbCommands;

  private final Command intakeAlgaeCommands;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (SimConstants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        csArm = new CoralScorerArm(new CoralScorerArmIOTalonFX(1));

        vision =
            new Vision(
                drive.getToPoseEstimatorConsumer(),
                new VisionIOLimelight("limelight 1", drive.getRawGyroRotationSupplier()),
                new VisionIOLimelight("limelight 2", drive.getRawGyroRotationSupplier()),
                new VisionIOLimelight("limelight 3", drive.getRawGyroRotationSupplier()),
                new VisionIOPhotonVision("photon", new Transform3d()));
        // TODO change lead, follower, gyro IDs, etc.
        elevator = new Elevator(new ElevatorIOTalonFX(0, 0));
        algaeArm = new AlgaeIntakeArm(new AlgaeIntakeArmIOTalonFX(0, 0, 0));
        csFlywheel =
            new CoralScorerFlywheel(
                new CoralScorerFlywheelIOSim(),
                new CoralIntakeSensorIO() {},
                CoralState.DEFAULT,
                AlgaeState.DEFAULT);
        led = new LED(new LED_IOCANdle(0, ""));
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
        algaeArm = new AlgaeIntakeArm(new AlgaeIntakeArmIOSim());
        csFlywheel =
            new CoralScorerFlywheel(
                new CoralScorerFlywheelIOSim(),
                new CoralIntakeSensorIO() {},
                CoralState.DEFAULT,
                AlgaeState.DEFAULT);
        led = new LED(new LED_IOSim());

        // objectDetection = new ObjectDetection(new ObjectDetectionConsumer() {}, new
        // ObjectDetectionIO() {});

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
        vision =
            new Vision(
                drive.getToPoseEstimatorConsumer(),
                new VisionIOLimelight("limelight 1", drive.getRawGyroRotationSupplier()),
                new VisionIOLimelight("limelight 2", drive.getRawGyroRotationSupplier()),
                new VisionIOLimelight("limelight 3", drive.getRawGyroRotationSupplier()),
                new VisionIOPhotonVision("photon", new Transform3d()));
        elevator = new Elevator(new ElevatorIO() {});
        algaeArm = new AlgaeIntakeArm(new AlgaeIntakeArmIOSim());
        csFlywheel =
            new CoralScorerFlywheel(
                new CoralScorerFlywheelIOSim(),
                new CoralIntakeSensorIO() {},
                CoralState.DEFAULT,
                AlgaeState.DEFAULT);
        led = new LED(new LED_IO() {});

        // objectDetection = new ObjectDetection(new ObjectDetectionConsumer() {}, new
        // ObjectDetectionIO() {});

        break;
    }

    climbStateMachine = new ClimbStateMachine(csArm);

    AlignToReefAuto alignToReef = new AlignToReefAuto(drive, led);
    intakeAlgaeCommands =
        new SelectCommand<>(
            Map.ofEntries(
                Map.entry(
                    ReefHeight.L1,
                    new IntakingAlgae(elevator, csFlywheel, csArm, FieldConstants.ReefHeight.L1)),
                Map.entry(
                    ReefHeight.L2,
                    new IntakingAlgae(elevator, csFlywheel, csArm, FieldConstants.ReefHeight.L2))),
            this::getNearestReefSideAndConvertToAlgaeHeight);
    // angles in none and retract aren't set, CHANGE THEM!!
    climbCommands =
        new SelectCommand<>(
            Map.ofEntries(
                Map.entry(
                    CLIMB_STATES.EXTEND,
                    algaeArm
                        .setArmTarget(160, 5)
                        .andThen(climbStateMachine::advanceTargetState, algaeArm)),
                Map.entry(
                    CLIMB_STATES.RETRACT,
                    algaeArm
                        .setArmTarget(65, 5)
                        .andThen(climbStateMachine::advanceTargetState, algaeArm)),
                Map.entry(
                    CLIMB_STATES.NONE,
                    algaeArm
                        .setArmTarget(65, 5)
                        .andThen(climbStateMachine::advanceTargetState, algaeArm))),
            this::climbSelect);

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

    NamedCommands.registerCommand("AlignToReefAuto", new AlignToReefAuto(drive, led));
    NamedCommands.registerCommand("AutoAlignToSource", new AutoAlignToSource(drive, led));
    NamedCommands.registerCommand(
        "IntakeFromSource", new IntakeFromSourceParallel(csFlywheel, csArm, elevator));
    NamedCommands.registerCommand(
        "IntakingAlgae", new IntakingAlgaeParallel(elevator, csArm, csFlywheel));
    NamedCommands.registerCommand("Stow", new Stow(csArm, elevator, led));

    // NamedCommands.registerCommand(
    // "AlgaeIntoProcessor", new AlgaeIntoProcessor(elevator, csArm, csFlywheel));
    // NamedCommands.registerCommand("ReadyForAlgaeScore", new ReadyForAlgaeScore(elevator, csArm));

    NamedCommands.registerCommand("ReleaseAlgae", new ReleaseAlgae(csFlywheel));

    NamedCommands.registerCommand("AutoPickupCoral", new AutoPickupCoral(null, drive, led));

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

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", autos);
    // Configure the button bindings
    // configureButtonBindings();
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void test() {
    keyboard
        .getXButton()
        .onTrue(new ReleaseClawParallel(ReefHeight.L2, elevator, csArm, csFlywheel));
    keyboard
        .getZButton()
        .onTrue(new ReleaseClawParallel(ReefHeight.L1, elevator, csArm, csFlywheel));

    keyboard
        .getXButton()
        .whileFalse(new ReleaseClawParallel(ReefHeight.L2, elevator, csArm, csFlywheel));
    keyboard
        .getZButton()
        .whileFalse(new ReleaseClawParallel(ReefHeight.L1, elevator, csArm, csFlywheel));
  }

  private void configureButtonBindings() {

    // controller.a().onTrue(new Stow(csArm, elevator));
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX(),
            () -> driveController.leftBumper().getAsBoolean(),
            () -> driveController.rightBumper().getAsBoolean()));
    driveController.leftBumper().onTrue(new InstantCommand(() -> drive.setNearestReefSide()));

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

    // controller.y().whileTrue(elevator.setElevatorTarget(1.83, 1));
    // controller.y().whileFalse(elevator.setElevatorTarget(1, 1));

    controller.x().whileTrue(csArm.setArmTarget(90, 1));
    controller.x().whileFalse(csArm.setArmTarget(-90, 1));

    controller.b().whileTrue(algaeArm.setArmTarget(70, 2));
    controller.b().whileFalse(algaeArm.setArmTarget(20, 2));

    manipController.rightBumper().onTrue(new IntakingAlgaeParallel(elevator, csArm, csFlywheel));
    manipController
        .rightBumper()
        .onFalse(
            new ParallelCommandGroup(
                csArm.setArmTarget(60, 4),
                elevator.setElevatorTarget(0.2, 0.05),
                new InstantCommand(() -> csFlywheel.runVolts(12))));

    manipController.rightTrigger().onTrue(new Stow(csArm, elevator, led));
    // driveController.a().whileTrue(new ReleaseClawParallel(scoringLevel, elevator, csArm,
    // csFlywheel));
    driveController.rightBumper().onTrue(new AlignToReefAuto(drive, led));

    driveController.leftBumper().onTrue(new AutoAlignToSource(drive, led));

    // driveController.rightTrigger().onTrue(new ReleaseAlgae(csFlywheel));

    // manipController.a().onTrue(new InstantCommand(() ->
    // elevator.setElevatorTarget(FieldConstants.ReefHeight.L1.height, 1)));
    // manipController.b().onTrue(new InstantCommand(() ->
    // elevator.setElevatorTarget(FieldConstants.ReefHeight.L2.height, 1)));
    // manipController.x().onTrue(new InstantCommand(() ->
    // elevator.setElevatorTarget(FieldConstants.ReefHeight.L3.height, 1)));
    // manipController.y().onTrue(new InstantCommand(() ->
    // elevator.setElevatorTarget(FieldConstants.ReefHeight.L4.height, 1)));

    manipController
        .a()
        .onTrue(new ReleaseClawParallel(FieldConstants.ReefHeight.L1, elevator, csArm, csFlywheel));
    manipController
        .b()
        .onTrue(new ReleaseClawParallel(FieldConstants.ReefHeight.L2, elevator, csArm, csFlywheel));

    driveController
        .a()
        .onTrue(new ReleaseClawParallel(FieldConstants.ReefHeight.L1, elevator, csArm, csFlywheel));
    driveController
        .b()
        .onTrue(new ReleaseClawParallel(FieldConstants.ReefHeight.L2, elevator, csArm, csFlywheel));
    driveController
        .x()
        .onTrue(new ReleaseClawParallel(FieldConstants.ReefHeight.L3, elevator, csArm, csFlywheel));
    driveController
        .y()
        .onTrue(new ReleaseClawParallel(FieldConstants.ReefHeight.L4, elevator, csArm, csFlywheel));

    controller.y().onTrue(climbCommands);

    driveController.leftTrigger().onTrue(new IntakingAlgaeParallel(elevator, csArm, csFlywheel));
    controller
        .leftBumper()
        .onFalse(
            new ParallelCommandGroup(
                csArm.setArmTarget(60, 4),
                elevator.setElevatorTarget(0.2, 0.05),
                new InstantCommand(() -> csFlywheel.runVolts(0))));
    controller
        .leftBumper()
        .onTrue(new ReleaseClawParallel(FieldConstants.ReefHeight.L4, elevator, csArm, csFlywheel));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous.
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
