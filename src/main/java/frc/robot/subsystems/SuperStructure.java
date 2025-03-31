package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.BargeExtend;
import frc.robot.commands.GoToStowAfterProcessor;
import frc.robot.commands.GoToStowTeleOp;
import frc.robot.commands.IntakeAlgaeFromReef;
import frc.robot.commands.IntakingCoral;
import frc.robot.commands.MoveToProcessorSetpoints;
import frc.robot.commands.ScoreAlgaeIntoBargeTele;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.SetElevatorTarget;
import frc.robot.commands.SetScoralArmTarget;
import frc.robot.commands.ToReefHeight;
import frc.robot.constants.SubsystemConstants;
import frc.robot.constants.SubsystemConstants.LED_STATE;
import frc.robot.constants.SubsystemConstants.ScoralArmConstants;
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
  private boolean algaeMode = false;

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
    } else if (wantedState == SuperStructureState.BARGE_EXTEND) {
      led.setState(LED_STATE.PINK_LAVENDER);
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

  public boolean shouldSlowMode() {
    return elevator.shouldSlowMode();
  }

  public boolean shouldWinch() {
    return climberArm.shouldWinch() && scoralArm.shouldWinch();
  }

  public boolean atGoals() {
    switch (currentState) {
      case STOW:
        return elevator.hasReachedGoal(SubsystemConstants.ElevatorConstants.STOW_SETPOINT_INCH)
            && scoralArm.hasReachedGoal(SubsystemConstants.ScoralArmConstants.STOW_SETPOINT_DEG);
      case L1:
        return elevator.hasReachedGoal(SubsystemConstants.ElevatorConstants.L1_SETPOINT_INCHES)
            && scoralArm.hasReachedGoal(SubsystemConstants.ScoralArmConstants.STOW_SETPOINT_DEG);
      case L2:
        return elevator.hasReachedGoal(SubsystemConstants.ElevatorConstants.L2_SETPOINT_INCHES)
            && scoralArm.hasReachedGoal(
                SubsystemConstants.ScoralArmConstants.L2_CORAL_SCORING_SETPOINT_DEG);
        // return true;
      case L3:
        return elevator.hasReachedGoal(SubsystemConstants.ElevatorConstants.L3_SETPOINT_INCHES)
            && scoralArm.hasReachedGoal(
                SubsystemConstants.ScoralArmConstants.L3_CORAL_SCORING_SETPOINT_DEG);
      case L4:
        return elevator.hasReachedGoal(SubsystemConstants.ElevatorConstants.L4_SETPOINT_INCHES)
            && scoralArm.hasReachedGoal(
                SubsystemConstants.ScoralArmConstants.L4_CORAL_SCORING_SETPOINT_DEG);
      case SOURCE:
        return elevator.hasReachedGoal(SubsystemConstants.ElevatorConstants.STOW_SETPOINT_INCH)
            && scoralArm.hasReachedGoal(SubsystemConstants.ScoralArmConstants.STOW_SETPOINT_DEG);
      case SCORING_CORAL:
        return true;
      case INTAKE_ALGAE:
        return true;
      case EJECT_ALGAE:
        return true;
      case BARGE_EXTEND:
        return elevator.hasReachedGoal(SubsystemConstants.ElevatorConstants.BARGE_SETPOINT)
            && scoralArm.hasReachedGoal(
                SubsystemConstants.ScoralArmConstants.BARGE_BACK_SETPOINT_DEG);
      case PROCESSOR:
        return elevator.hasReachedGoal(4) && scoralArm.hasReachedGoal(20);
      case PROCESSOR_SCORE:
        return true;
      case BARGE_SCORE:
        return true;
      default:
        return false;
    }

    // TODO:: COMMENT
    // return true;
  }

  public void enableAlgaeMode(boolean bool) {
    algaeMode = bool;
  }

  public boolean getAlgaeMode() {
    return algaeMode;
  }

  public SuperStructureState getLastReefState() {
    return lastReefState;
  }

  public SequentialCommandGroup getSuperStructureCommand() {
    counter++;
    Logger.recordOutput("Debug Super Structure/counter", counter);
    switch (wantedState) {
      case STOW:
        SequentialCommandGroup command;
        led.setState(LED_STATE.BLUE);
        currentState = SuperStructureState.STOW;
        if (elevator.getElevatorPosition() <= 20) {
          command = new GoToStowAfterProcessor(elevator, scoralArm, scoralRollers);
        } else {
          command = new GoToStowTeleOp(elevator, scoralArm, scoralRollers);
        }
        return command.andThen(new InstantCommand(() -> nextState()));
      case EJECT_ALGAE:
        led.setState(LED_STATE.BLUE);
        currentState = SuperStructureState.EJECT_ALGAE;
        return new SequentialCommandGroup(
            new InstantCommand(() -> scoralRollers.runVolts(2))
            // new ToReefHeight(
            //     elevator,
            //     scoralArm,
            //     ElevatorConstants.L2_SETPOINT_INCHES,
            //     ScoralArmConstants.L3_CORAL_SCORING_SETPOINT_DEG),
            );
      case BARGE_EXTEND:
        currentState = SuperStructureState.BARGE_EXTEND;
        return new BargeExtend(elevator, scoralArm);
      case BARGE_SCORE:
        currentState = SuperStructureState.BARGE_SCORE;
        led.setState(LED_STATE.FLASHING_GREEN);
        return new SequentialCommandGroup(
            new ScoreAlgaeIntoBargeTele(elevator, scoralArm, scoralRollers),
            new InstantCommand(() -> led.setState(LED_STATE.BLUE)),
            new InstantCommand(() -> this.setCurrentState(SuperStructureState.STOW)),
            new InstantCommand(() -> this.setWantedState(SuperStructureState.STOW)));
      case PROCESSOR_SCORE:
        currentState = SuperStructureState.PROCESSOR_SCORE;
        led.setState(LED_STATE.FLASHING_GREEN);
        return new SequentialCommandGroup(
            scoralRollers.runVoltsCommmand(2),
            new WaitCommand(0.5),
            new GoToStowAfterProcessor(elevator, scoralArm, scoralRollers),
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
            SubsystemConstants.ScoralArmConstants.L2_CORAL_SCORING_SETPOINT_DEG);
        // return new SequentialCommandGroup(
        // new SetElevatorTarget(elevator, 6.8, 2), new SetScoralArmTarget(scoralArm, 71, 2));

      case L3:
        currentState = SuperStructureState.L3;
        lastReefState = SuperStructureState.L3;
        return new ToReefHeight(
            elevator,
            scoralArm,
            SubsystemConstants.ElevatorConstants.L3_SETPOINT_INCHES,
            SubsystemConstants.ScoralArmConstants.L3_CORAL_SCORING_SETPOINT_DEG);

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
        return new SetScoralArmTarget(
                scoralArm, SubsystemConstants.ScoralArmConstants.STOW_SETPOINT_DEG, 2)
            .andThen(new IntakingCoral(scoralRollers))
            .andThen(
                new InstantCommand(() -> led.setState(LED_STATE.BLUE))
                    .andThen(
                        new InstantCommand(() -> this.setCurrentState(SuperStructureState.STOW))
                            .andThen(new InstantCommand(() -> this.nextState()))));

      case PROCESSOR:
        currentState = SuperStructureState.PROCESSOR;
        return new SequentialCommandGroup(new MoveToProcessorSetpoints(scoralArm, elevator));

      case SCORING_CORAL:
        currentState = SuperStructureState.SCORING_CORAL;
        led.setState(LED_STATE.FLASHING_GREEN);
        SequentialCommandGroup stowCommand;
        if (elevator.getElevatorPosition() <= 20) {
          stowCommand = new GoToStowAfterProcessor(elevator, scoralArm, scoralRollers);
        } else {
          stowCommand = new GoToStowTeleOp(elevator, scoralArm, scoralRollers);
        }
        if (!algaeMode) {
          return new SequentialCommandGroup(
              new ScoreCoral(elevator, scoralArm, scoralRollers),
              stowCommand,
              new InstantCommand(() -> led.setState(LED_STATE.BLUE)),
              new InstantCommand(() -> this.setCurrentState(SuperStructureState.STOW)),
              new InstantCommand(() -> this.setWantedState(SuperStructureState.STOW)));
        } else {

          double height1 =
              drive.getNearestParition(6) % 2 == 0
                  ? 6.5
                  : SubsystemConstants.ElevatorConstants.STOW_SETPOINT_INCH;
          double height2 = drive.getNearestParition(6) % 2 == 0 ? 9 : 2;

          Command scoralArmCommand;
          if (elevator.getElevatorPosition() >= 20) {
            scoralArmCommand =
                new SetScoralArmTarget(scoralArm, ScoralArmConstants.STOW_SETPOINT_DEG + 4, 2);
          } else {
            scoralArmCommand =
                new SetScoralArmTarget(scoralArm, ScoralArmConstants.STOW_SETPOINT_DEG - 4, 2);
          }
          // double height1 = drive.getNearestParition(6) % 2 == 0 ? 6 : 6.5;
          // double height2 = drive.getNearestParition(6) % 2 == 0 ? 8 : 9;

          return new SequentialCommandGroup(
              scoralRollers.runVoltsCommmand(2.6),
              new WaitCommand(0.5),
              scoralArmCommand,
              scoralRollers.stopCommand(),
              new InstantCommand(() -> this.setCurrentState(SuperStructureState.INTAKE_ALGAE)),
              new InstantCommand(() -> this.setWantedState(SuperStructureState.EJECT_ALGAE)),
              new IntakeAlgaeFromReef(
                  drive, scoralArm, scoralRollers, elevator, led, height1, height2),
              new InstantCommand(() -> this.enableAlgaeMode(false)));
        }

      case INTAKE_ALGAE:
        currentState = SuperStructureState.INTAKE_ALGAE;
        double height1 =
            drive.getNearestParition(6) % 2 == 0
                ? 6.5
                : SubsystemConstants.ElevatorConstants.STOW_SETPOINT_INCH;
        double height2 = drive.getNearestParition(6) % 2 == 0 ? 9 : 2;
        return new IntakeAlgaeFromReef(
            drive, scoralArm, scoralRollers, elevator, led, height1, height2);

      default:
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SetElevatorTarget(elevator, 0, 0), new SetScoralArmTarget(scoralArm, 40, 0)));
    }
  }

  public void nextState() {
    switch (currentState) {
        // case NONE:
        // break;
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
      case SCORING_CORAL, PROCESSOR_SCORE, EJECT_ALGAE:
        setWantedState(SuperStructureState.STOW);
        break;
      case INTAKE_ALGAE:
        setWantedState(SuperStructureState.EJECT_ALGAE);
        break;
      case PROCESSOR:
        setWantedState(SuperStructureState.PROCESSOR_SCORE);
        break;
      case BARGE_EXTEND:
        setWantedState(SuperStructureState.BARGE_SCORE);
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

  public boolean isCurrentAReefState() {
    return currentState == SuperStructureState.L1
        || currentState == SuperStructureState.L2
        || currentState == SuperStructureState.L3
        || currentState == SuperStructureState.L4;
  }
}
