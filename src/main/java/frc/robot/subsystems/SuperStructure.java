package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.ScoringProccessorSequential;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.SubsystemConstants;
import frc.robot.constants.SubsystemConstants.CoralState;
import frc.robot.constants.SubsystemConstants.LED_STATE;
import frc.robot.constants.SubsystemConstants.SuperStructureState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.scoral.ScoralArm;
import frc.robot.subsystems.scoral.ScoralRollers;
import org.littletonrobotics.junction.Logger;

public class SuperStructure {
  private final Drive drive;
  private final Elevator elevator;
  private final ScoralArm scoralArm;
  private final ScoralRollers scoralRollers;
  private final LED led;
  private SuperStructureState currentState;
  private SuperStructureState wantedState;
  public boolean override = false;
  int counter = 0;

  public SuperStructure(
      Drive drive, Elevator elevator, ScoralArm scoralArm, ScoralRollers scoralRollers, LED led) {
    this.drive = drive;
    this.elevator = elevator;
    this.scoralArm = scoralArm;
    this.scoralRollers = scoralRollers;
    this.led = led;
    wantedState = SuperStructureState.STOW;
    currentState = SuperStructureState.STOW;
  }

  public void setWantedState(SuperStructureState wantedState) {
    if (wantedState == SuperStructureState.L1
        || wantedState == SuperStructureState.L2
        || wantedState == SuperStructureState.L3
        || wantedState == SuperStructureState.L4) {
      led.setState(LED_STATE.RED);
    } else if (wantedState == SuperStructureState.SOURCE) {
      led.setState(LED_STATE.GREY);
    } else if (wantedState == SuperStructureState.PROCESSOR) {
      led.setState(LED_STATE.YELLOW);
    } else if (wantedState == SuperStructureState.CLIMB_STAGE_ONE) {
      led.setState(LED_STATE.PAPAYA_ORANGE);
    }
    this.wantedState = wantedState;
  }

  public void setCurrentState(SuperStructureState currentState) {
    this.currentState = currentState;
  }

  public SuperStructureState getWantedState() {
    return wantedState;
  }

  public SuperStructureState getCurrentState() {
    return currentState;
  }

  public boolean elevatorExtended() {
    return elevator.isExtended();
  }

  public boolean atGoals() {
    // switch (currentState) {
    // case STOW:
    //   return elevator.hasReachedGoal(0) && csArm.hasReachedGoal(40);
    // case L1:
    //   return elevator.hasReachedGoal(FieldConstants.ReefHeight.L1.height)
    //       && csArm.hasReachedGoal(FieldConstants.ReefHeight.L1.pitch);
    // case L2:
    //   return elevator.hasReachedGoal(FieldConstants.ReefHeight.L2.height)
    //       && csArm.hasReachedGoal(FieldConstants.ReefHeight.L2.pitch);
    // case L3:
    //   return elevator.hasReachedGoal(FieldConstants.ReefHeight.L3.height)
    //       && csArm.hasReachedGoal(FieldConstants.ReefHeight.L3.pitch);
    // case L4:
    //   return elevator.hasReachedGoal(FieldConstants.ReefHeight.L4.height)
    //       && csArm.hasReachedGoal(FieldConstants.ReefHeight.L4.pitch);
    // case SOURCE:
    //   return elevator.hasReachedGoal(0) && csArm.hasReachedGoal(40);
    // case SCORING_CORAL:
    //   // return csFlywheel.seesCoral() == CoralState.CURRENT
    //   // || csFlywheel.seesCoral() == CoralState.SENSOR;
    //   return true;
    // default:
    //   return false;
    // }
    return true;
  }

  public SequentialCommandGroup getSuperStructureCommand() {
    counter++;
    Logger.recordOutput("bruhufe", counter);
    switch (wantedState) {
      case STOW:
        led.setState(LED_STATE.BLUE);
        currentState = SuperStructureState.STOW;
        return new SequentialCommandGroup(
            scoralRollers.stopCommand(),
            new ParallelCommandGroup(
                elevator.setElevatorTarget(
                    SubsystemConstants.ElevatorConstants.STOW_SETPOINT_INCH, 2),
                scoralArm.setArmTarget(
                    SubsystemConstants.CoralScorerConstants.ScoralArmConstants.STOW_SETPOINT_DEG,
                    2),
                scoralRollers.stopCommand(),
                led.setStateCommand(LED_STATE.BLUE),
                new InstantCommand(() -> System.out.println("the stow command has been run"))));

      case INTAKE_ALGAE:
        double height =
            drive.getNearestParition(6) % 2 == 0
                ? FieldConstants.ReefHeight.L4.height
                : FieldConstants.ReefHeight.L4.height;

        currentState = SuperStructureState.INTAKE_ALGAE;
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                elevator.setElevatorTarget(height - 1, 0.1),
                scoralArm.setArmTarget(30, 2),
                led.setStateCommand(LED_STATE.FLASHING_GREEN)));

      case L1:
        currentState = SuperStructureState.L1;
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                elevator.setElevatorTarget(FieldConstants.ReefHeight.L1.height, 0.1),
                scoralArm.setArmTarget(FieldConstants.ReefHeight.L1.pitch, 2),
                led.setStateCommand(LED_STATE.FLASHING_GREEN)));

      case L2:
        currentState = SuperStructureState.L2;
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                elevator.setElevatorTarget(FieldConstants.ReefHeight.L2.height, 0.1),
                scoralArm.setArmTarget(FieldConstants.ReefHeight.L2.pitch, 2)));
      case L3:
        currentState = SuperStructureState.L3;
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                elevator.setElevatorTarget(FieldConstants.ReefHeight.L3.height, 0.1),
                scoralArm.setArmTarget(FieldConstants.ReefHeight.L3.pitch, 2)));
      case L4:
        currentState = SuperStructureState.L4;
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                elevator.setElevatorTarget(FieldConstants.ReefHeight.L4.height, 0.1),
                scoralArm.setArmTarget(FieldConstants.ReefHeight.L4.pitch, 2)));

      case SOURCE:
        currentState = SuperStructureState.SOURCE;
        return new SequentialCommandGroup(
            scoralRollers.runVoltsCommmand(5),
            new WaitUntilCommand(
                () ->
                    scoralRollers.seesCoral() == CoralState.CURRENT
                        || scoralRollers.seesCoral() == CoralState.SENSOR),
            new WaitCommand(0.5),
            new InstantCommand(() -> scoralRollers.stop()),
            new InstantCommand(() -> led.setState(LED_STATE.BLUE)),
            new InstantCommand(() -> this.setCurrentState(SuperStructureState.STOW)),
            new InstantCommand(() -> this.setWantedState(SuperStructureState.STOW)));
      case PROCESSOR:
        led.setState(LED_STATE.FLASHING_GREEN);
        currentState = SuperStructureState.PROCESSOR;
        return new ScoringProccessorSequential(scoralRollers, scoralArm, elevator);

      case SCORING_CORAL:
        currentState = SuperStructureState.SCORING_CORAL;
        led.setState(LED_STATE.FLASHING_GREEN);
        return new SequentialCommandGroup(
            scoralRollers.runVoltsCommmand(1),
            new WaitUntilCommand(() -> scoralRollers.seesCoral() == CoralState.NO_CORAL),
            new ParallelCommandGroup(
                elevator.setElevatorTarget(0, 2),
                scoralArm.setArmTarget(40, 2),
                scoralRollers.stopCommand(),
                led.setStateCommand(LED_STATE.BLUE),
                new InstantCommand(() -> System.out.println("the stow command has been run"))),
            new InstantCommand(() -> this.setCurrentState(SuperStructureState.STOW)),
            new InstantCommand(() -> this.setWantedState(SuperStructureState.STOW)));
      case CLIMB_STAGE_ONE:
        return new SequentialCommandGroup();
      case CLIMB_STAGE_TWO:
        return new SequentialCommandGroup();
      case HANG:
        led.setState(LED_STATE.BLUE);
        return new SequentialCommandGroup();
      default:
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                elevator.setElevatorTarget(0, 0), scoralArm.setArmTarget(40, 0)));
    }
  }

  public void advanceWantedState() {
    switch (currentState) {
        // case NONE:
        //   break;
      case STOW:
        break;
      case L1, L2, L3, L4:
        setWantedState(SuperStructureState.SCORING_CORAL);
        break;
      case SCORING_CORAL, SOURCE, PROCESSOR, INTAKE_ALGAE:
        setWantedState(SuperStructureState.STOW);
        break;
      case CLIMB_STAGE_ONE:
        setWantedState(SuperStructureState.CLIMB_STAGE_TWO);
        break;
      case CLIMB_STAGE_TWO:
        setWantedState(SuperStructureState.HANG);
        break;
      case HANG:
        setWantedState(SuperStructureState.CLIMB_STAGE_ONE);
        break;

      default:
        break;
    }
  }

  public boolean isTargetAReefState() {
    return wantedState == SuperStructureState.L1
        || wantedState == SuperStructureState.L2
        || wantedState == SuperStructureState.L3
        || wantedState == SuperStructureState.L4;
  }
}
