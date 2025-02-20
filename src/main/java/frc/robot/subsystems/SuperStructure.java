package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.SubsystemConstants.CoralState;
import frc.robot.constants.SubsystemConstants.LED_STATE;
import frc.robot.constants.SubsystemConstants.SuperStructureState;
import frc.robot.subsystems.coralscorer.CoralScorerArm;
import frc.robot.subsystems.coralscorer.CoralScorerFlywheel;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.led.LED;
import org.littletonrobotics.junction.Logger;

public class SuperStructure {
  private final Drive drive;
  private final Elevator elevator;
  private final CoralScorerArm csArm;
  private final CoralScorerFlywheel csFlywheel;
  private final LED led;
  private SuperStructureState currentState;
  private SuperStructureState wantedState;
  public boolean override = false;
  int counter = 0;

  public SuperStructure(
      Drive drive,
      Elevator elevator,
      CoralScorerArm csArm,
      CoralScorerFlywheel csFlywheel,
      LED led) {
    this.drive = drive;
    this.elevator = elevator;
    this.csArm = csArm;
    this.csFlywheel = csFlywheel;
    this.led = led;
    wantedState = SuperStructureState.STOW;
    currentState = SuperStructureState.STOW;
  }

  public void setWantedState(SuperStructureState wantedState) {
    this.wantedState = wantedState;
    // this.currentState = wantedState;
  }

  // public boolean isRobotTooFast() {
  //   return Drive.chassisSpeedMetersPerSec > 2 || Drive.rotationVelocityDegsPerSec > 50;
  //   // if (Drive.chassisSpeedMetersPerSec > 2 || Drive.rotationVelocityDegsPerSec > 50) {
  //   //   this.wantedState = SuperStructureState.STOW;
  //   // }
  //   // else {

  //   //   currentState = wantedState;
  //   // }
  // }

  public SuperStructureState getWantedState() {
    return wantedState;
  }

  public SuperStructureState getCurrentState() {
    return currentState;
  }

  // public boolean changedStated() {

  //   return currentState != wantedState;
  // }

  public boolean elevatorExtended() {
    return elevator.isExtended();
  }

  public boolean atGoals() {
    switch (currentState) {
      case STOW:
        return elevator.hasReachedGoal(0) && csArm.hasReachedGoal(40);
      case L1:
        return elevator.hasReachedGoal(FieldConstants.ReefHeight.L1.height)
            && csArm.hasReachedGoal(FieldConstants.ReefHeight.L1.pitch);
      case L2:
        return elevator.hasReachedGoal(FieldConstants.ReefHeight.L2.height)
            && csArm.hasReachedGoal(FieldConstants.ReefHeight.L2.pitch);
      case L3:
        return elevator.hasReachedGoal(FieldConstants.ReefHeight.L3.height)
            && csArm.hasReachedGoal(FieldConstants.ReefHeight.L3.pitch);
      case L4:
        return elevator.hasReachedGoal(FieldConstants.ReefHeight.L4.height)
            && csArm.hasReachedGoal(FieldConstants.ReefHeight.L4.pitch);
      case SOURCE:
        return elevator.hasReachedGoal(0) && csArm.hasReachedGoal(40);
      case SCORING_CORAL:
        // return csFlywheel.seesCoral() == CoralState.CURRENT
        //     || csFlywheel.seesCoral() == CoralState.SENSOR;
        return true;
      default:
        return false;
    }
  }

  public SequentialCommandGroup getSuperStructureCommand() {
    counter++;
    Logger.recordOutput("bruhufe", counter);
    switch (wantedState) {
      case STOW:
        currentState = SuperStructureState.STOW;
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                elevator.setElevatorTarget(0, 2),
                csArm.setArmTarget(40, 2),
                csFlywheel.stopCommand(),
                led.setStateCommand(LED_STATE.BLUE)));

      case INTAKE_ALGAE:
        double height =
            drive.getNearestParition(6) % 2 == 0
                ? FieldConstants.ReefHeight.L4.height
                : FieldConstants.ReefHeight.L4.height;

        currentState = SuperStructureState.INTAKE_ALGAE;
        // if (getNearestParition(6)%2 == 0 ? high pos : low pos) {

        // }
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                elevator.setElevatorTarget(height - 1, 0.1),
                csArm.setArmTarget(30, 2),
                led.setStateCommand(LED_STATE.FLASHING_GREEN)));

        // return new SequentialCommandGroup(
        //     new ParallelCommandGroup(
        //         elevator.setElevatorTarget(FieldConstants.ReefHeight.L4.height - 1, 0.1),
        //         csArm.setArmTarget(30, 2),
        //         led.setStateCommand(LED_STATE.FLASHING_GREEN)));

      case L1:
        currentState = SuperStructureState.L1;
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                elevator.setElevatorTarget(FieldConstants.ReefHeight.L1.height, 0.1),
                csArm.setArmTarget(FieldConstants.ReefHeight.L1.pitch, 2),
                led.setStateCommand(LED_STATE.FLASHING_GREEN)));
        //  csFlywheel.runVoltsCommmand(12),
        // new WaitCommand(1),
        // led.setStateCommand(LED_STATE.GREEN)

      case L2:
        currentState = SuperStructureState.L2;
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                elevator.setElevatorTarget(FieldConstants.ReefHeight.L2.height, 0.1),
                csArm.setArmTarget(FieldConstants.ReefHeight.L2.pitch, 2)));
      case L3:
        currentState = SuperStructureState.L3;
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                elevator.setElevatorTarget(FieldConstants.ReefHeight.L3.height, 0.1),
                csArm.setArmTarget(FieldConstants.ReefHeight.L3.pitch, 2)));
      case L4:
        currentState = SuperStructureState.L4;
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                elevator.setElevatorTarget(FieldConstants.ReefHeight.L4.height, 0.1),
                csArm.setArmTarget(FieldConstants.ReefHeight.L4.pitch, 2)));

      case SOURCE:
        currentState = SuperStructureState.SOURCE;
        if (csFlywheel.seesCoral() == CoralState.CURRENT
            || csFlywheel.seesCoral() == CoralState.SENSOR) {
          return new SequentialCommandGroup(
              new WaitCommand(0.5),
              new InstantCommand(() -> setWantedState(SuperStructureState.STOW)));

        } else {
          return new SequentialCommandGroup(
              new ParallelCommandGroup(
                  elevator.setElevatorTarget(1, 0.1),
                  csArm.setArmTarget(FieldConstants.ReefHeight.L2.pitch, 2)),
              csFlywheel.runVelocityCommand(200));
        }

      case SCORING_CORAL:
        currentState = SuperStructureState.SCORING_CORAL;
        // if (csFlywheel.seesCoral() == CoralState.SENSOR
        // || csFlywheel.seesCoral() == CoralState.CURRENT) {
        return new SequentialCommandGroup(csFlywheel.runVoltsCommmand(1));
        // } else {
        //   return new SequentialCommandGroup(
        //       new WaitCommand(0.5),
        //       new ParallelCommandGroup(
        //           elevator.setElevatorTarget(0, 0),
        //           csArm.setArmTarget(40, 0),
        //           csFlywheel.stopCommand(),
        //           led.setStateCommand(LED_STATE.BLUE)));
        // }

      default:
        return new SequentialCommandGroup(
            new ParallelCommandGroup(elevator.setElevatorTarget(0, 0), csArm.setArmTarget(40, 0)));
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
