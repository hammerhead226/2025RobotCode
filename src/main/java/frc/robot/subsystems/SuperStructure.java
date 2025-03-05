package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.GoToStow;
import frc.robot.commands.IntakeAlgae;
import frc.robot.commands.IntakingCoral;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.ScoringProccessorSequential;
import frc.robot.commands.ToReefHeight;
import frc.robot.constants.SubsystemConstants;
import frc.robot.constants.SubsystemConstants.LED_STATE;
import frc.robot.constants.SubsystemConstants.SuperStructureState;
import frc.robot.subsystems.climber.ClimberArm;
import frc.robot.subsystems.climber.Winch;
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
  private final ClimberArm climberArm;
  private final Winch winch;
  private SuperStructureState currentState;
  private SuperStructureState wantedState;
  private SuperStructureState lastReefState;
  public boolean override = false;
  int counter = 0;

  public SuperStructure(
      Drive drive,
      Elevator elevator,
      ScoralArm scoralArm,
      ScoralRollers scoralRollers,
      LED led,
      ClimberArm climberArm,
      Winch winch) {
    this.drive = drive;
    this.elevator = elevator;
    this.scoralArm = scoralArm;
    this.scoralRollers = scoralRollers;
    this.led = led;
    this.climberArm = climberArm;
    this.winch = winch;
    wantedState = SuperStructureState.STOW;
    currentState = SuperStructureState.STOW;
    lastReefState = SuperStructureState.L4;
  }

  public void setWantedState(SuperStructureState wantedState) {
    if (wantedState == SuperStructureState.L1
        || wantedState == SuperStructureState.L2
        || wantedState == SuperStructureState.L3
        || wantedState == SuperStructureState.L4) {
      led.setState(LED_STATE.RED);
      lastReefState = wantedState;
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
    // if (currentState == SuperStructureState.STOW) {
    //   led.setState(LED_STATE.BLUE);
    // }
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
    if (SubsystemConstants.coralStuckMode) {
      return true;
    } else {
      switch (currentState) {
        case STOW:
          return elevator.hasReachedGoal(SubsystemConstants.ElevatorConstants.STOW_SETPOINT_INCH)
              && scoralArm.hasReachedGoal(SubsystemConstants.ScoralArmConstants.STOW_SETPOINT_DEG);
        case L1:
          return elevator.hasReachedGoal(SubsystemConstants.ElevatorConstants.L1_SETPOINT_INCHES)
              && scoralArm.hasReachedGoal(
                  SubsystemConstants.ScoralArmConstants.LOW_CORAL_SCORING_SETPOINT_DEG);
        case L2:
          return elevator.hasReachedGoal(SubsystemConstants.ElevatorConstants.L2_SETPOINT_INCHES)
              && scoralArm.hasReachedGoal(
                  SubsystemConstants.ScoralArmConstants.LOW_CORAL_SCORING_SETPOINT_DEG);
        case L3:
          return elevator.hasReachedGoal(SubsystemConstants.ElevatorConstants.L3_SETPOINT_INCHES)
              && scoralArm.hasReachedGoal(
                  SubsystemConstants.ScoralArmConstants.LOW_CORAL_SCORING_SETPOINT_DEG);
        case L4:
          return elevator.hasReachedGoal(SubsystemConstants.ElevatorConstants.L4_SETPOINT_INCHES)
              && scoralArm.hasReachedGoal(
                  SubsystemConstants.ScoralArmConstants.L4_CORAL_SCORING_SETPOINT_DEG);
        case SOURCE:
          return elevator.hasReachedGoal(SubsystemConstants.ElevatorConstants.STOW_SETPOINT_INCH)
              && scoralArm.hasReachedGoal(SubsystemConstants.ScoralArmConstants.STOW_SETPOINT_DEG);
        case SCORING_CORAL:
          // TODO:: UNCOMMENT
          // return scoralRollers.seesCoral() == CoralState.CURRENT
          // || scoralRollers.seesCoral() == CoralState.SENSOR;
          return true;
        case INTAKE_ALGAE:
          return true;
        case STOW_ALGAE:
          return true;
        case BARGE_EXTEND:
          return elevator.hasReachedGoal(SubsystemConstants.ElevatorConstants.BARGE_SETPOINT)
              && scoralArm.hasReachedGoal(SubsystemConstants.ScoralArmConstants.BARGE_SETPOINT);
        case PROCESSOR:
          return elevator.hasReachedGoal(4) && scoralArm.hasReachedGoal(20);
        case ALGAE_SCORE:
          return true;
          // case CLIMB_STAGE_ONE:
          //   // return climberArm.atGoal(SubsystemConstants.ClimberConstants.STOW_SETPOINT_DEG) &&
          //   //
          // scoralArm.hasReachedGoal(SubsystemConstants.ScoralArmConstants.STOW_SETPOINT_DEG);
          //   return true;
          // case CLIMB_STAGE_TWO:
          //   // return climberArm.atGoal(60);
          //   return true;
          // case HANG:
          //   return false;
        default:
          return false;
      }
    }

    // TODO:: COMMENT
    // return true;
  }

  public void toggleCoralStuckMode() {
    if (!SubsystemConstants.coralStuckMode) {
      SubsystemConstants.coralStuckMode = true;
    } else {
      SubsystemConstants.coralStuckMode = false;
    }
  }

  public SequentialCommandGroup getSuperStructureCommand() {
    counter++;
    Logger.recordOutput("Debug Super Structure/counter", counter);
    switch (wantedState) {
      case STOW:
        led.setState(LED_STATE.BLUE);
        currentState = SuperStructureState.STOW;
        return new GoToStow(elevator, scoralArm, scoralRollers);
        // .andThen(
        //     climberArm.setArmTarget(SubsystemConstants.ClimberConstants.STOW_SETPOINT_DEG, 2));

      case INTAKE_ALGAE:
        double height = drive.getNearestParition(6) % 2 == 0 ? 16.5 : 7.9;
        // double height = drive.getNearestParition(6) % 2 == 0 ? 7.9 : 16.5;

        currentState = SuperStructureState.INTAKE_ALGAE;
        return new IntakeAlgae(elevator, scoralArm, scoralRollers, height);

      case STOW_ALGAE:
        return new ToReefHeight(
            elevator,
            scoralArm,
            SubsystemConstants.ElevatorConstants.L2_SETPOINT_INCHES,
            SubsystemConstants.ScoralArmConstants.LOW_CORAL_SCORING_SETPOINT_DEG);
      case BARGE_EXTEND:
        currentState = SuperStructureState.BARGE_EXTEND;
        return new SequentialCommandGroup(
            elevator.setElevatorTarget(SubsystemConstants.ElevatorConstants.BARGE_SETPOINT, 2),
            new WaitUntilCommand(() -> elevator.atGoal(2)),
            scoralArm.setArmTarget(SubsystemConstants.ScoralArmConstants.BARGE_SETPOINT, 2));
      case ALGAE_SCORE:
        currentState = SuperStructureState.ALGAE_SCORE;
        led.setState(LED_STATE.FLASHING_GREEN);
        return new SequentialCommandGroup(
            scoralRollers.runVoltsCommmand(4),
            new WaitCommand(0.5),
            new GoToStow(elevator, scoralArm, scoralRollers),
            new InstantCommand(() -> led.setState(LED_STATE.BLUE)),
            new InstantCommand(() -> this.setCurrentState(SuperStructureState.STOW)),
            new InstantCommand(() -> this.setWantedState(SuperStructureState.STOW)));
      case L1:
        currentState = SuperStructureState.L1;
        lastReefState = SuperStructureState.L1;
        return new ToReefHeight(
            elevator,
            scoralArm,
            SubsystemConstants.ElevatorConstants.L1_SETPOINT_INCHES,
            SubsystemConstants.ScoralArmConstants.STOW_SETPOINT_DEG);

      case L2:
        currentState = SuperStructureState.L2;
        lastReefState = SuperStructureState.L2;
        return new ToReefHeight(
            elevator,
            scoralArm,
            SubsystemConstants.ElevatorConstants.L2_SETPOINT_INCHES,
            SubsystemConstants.ScoralArmConstants.LOW_CORAL_SCORING_SETPOINT_DEG);

      case L3:
        currentState = SuperStructureState.L3;
        lastReefState = SuperStructureState.L3;
        return new ToReefHeight(
            elevator,
            scoralArm,
            SubsystemConstants.ElevatorConstants.L3_SETPOINT_INCHES,
            SubsystemConstants.ScoralArmConstants.LOW_CORAL_SCORING_SETPOINT_DEG);

      case L4:
        currentState = SuperStructureState.L4;
        lastReefState = SuperStructureState.L4;
        return new ToReefHeight(
            elevator,
            scoralArm,
            SubsystemConstants.ElevatorConstants.L4_SETPOINT_INCHES,
            SubsystemConstants.ScoralArmConstants.L4_CORAL_SCORING_SETPOINT_DEG);

      case SOURCE:
        currentState = SuperStructureState.SOURCE;
        return new IntakingCoral(scoralRollers)
            .andThen(
                new InstantCommand(() -> led.setState(LED_STATE.BLUE))
                    .andThen(
                        new InstantCommand(() -> this.setCurrentState(SuperStructureState.STOW))
                            .andThen(new InstantCommand(() -> this.nextState()))));

      case PROCESSOR:
        led.setState(LED_STATE.FLASHING_GREEN);
        currentState = SuperStructureState.PROCESSOR;
        return new ScoringProccessorSequential(scoralArm, elevator);

      case SCORING_CORAL:
        currentState = SuperStructureState.SCORING_CORAL;
        led.setState(LED_STATE.FLASHING_GREEN);
        return new SequentialCommandGroup(
            new ScoreCoral(elevator, scoralArm, scoralRollers),
            new InstantCommand(() -> led.setState(LED_STATE.BLUE)),
            new InstantCommand(() -> this.setCurrentState(SuperStructureState.STOW)),
            new InstantCommand(() -> this.setWantedState(SuperStructureState.STOW)));

      default:
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                elevator.setElevatorTarget(0, 0), scoralArm.setArmTarget(40, 0)));
    }
  }

  public void nextState() {
    switch (currentState) {
        // case NONE:
        //   break;
      case STOW:
        if (scoralRollers.getDistance() <= SubsystemConstants.CORAL_DIST) {
          setWantedState(lastReefState);
        } else {
          setWantedState(SuperStructureState.SOURCE);
        }
        break;
      case SOURCE:
        setWantedState(lastReefState);
        break;
      case L1, L2, L3, L4:
        setWantedState(SuperStructureState.SCORING_CORAL);
        break;
      case SCORING_CORAL, ALGAE_SCORE:
        setWantedState(SuperStructureState.STOW);
        break;
      case INTAKE_ALGAE:
        setWantedState(SuperStructureState.STOW_ALGAE);
        break;
      case PROCESSOR:
        setWantedState(SuperStructureState.ALGAE_SCORE);
        break;
      case CLIMB_STAGE_ONE:
        setWantedState(SuperStructureState.CLIMB_STAGE_TWO);
        break;
      case CLIMB_STAGE_TWO:
        setWantedState(SuperStructureState.CLIMB_STAGE_THREE);
        break;
      case CLIMB_STAGE_THREE:
        setWantedState(SuperStructureState.HANG);
        break;
      case HANG:
        break;
      case BARGE_EXTEND:
        setWantedState(SuperStructureState.ALGAE_SCORE);
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
