// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.statemachines;

// import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.coralscorer.CoralScorerArm;
import org.littletonrobotics.junction.Logger;

public class ClimbStateMachine {

  private final CoralScorerArm csArm;

  public ClimbStateMachine(CoralScorerArm csArm) {
    this.csArm = csArm;
  }

  // public enum CLIMB_STATES {
  //   NONE,
  //   PIVOT_CLIMB,
  //   RETRACT_CLIMB,
  //   ENGAGE_STATIC_HOOKS,
  //   ENGAGE_LOWER_SHOOTER_HOOKS,
  //   ENGAGE_LOWER_SHOOTER_HOOKS_PART_TWO,
  //   ALIGN_TO_TRAP,
  //   SHOOT_NOTE,
  //   DONE
  // }

  public enum CLIMB_STATES {
    EXTEND,
    RETRACT,
    NONE
  }

  private CLIMB_STATES targetState = CLIMB_STATES.NONE;

  public CLIMB_STATES getTargetState() {
    return targetState;
  }

  public void setClimbState(CLIMB_STATES state) {
    targetState = state;
  }

  public void advanceTargetState() {
    switch (targetState) {
      case NONE:
        targetState = CLIMB_STATES.EXTEND;
        break;
        // case PIVOT:
        //   targetState = CLIMB_STATES.EXTEND;
        //   break;
      case EXTEND:
        targetState = CLIMB_STATES.RETRACT;
        break;
      case RETRACT:
        targetState = CLIMB_STATES.NONE;
        break;
      default:
        targetState = CLIMB_STATES.NONE;
        break;
    }
    Logger.recordOutput("Climb Target State", targetState);
  }

  // public void goBackState() {
  //   switch (targetState) {
  //     case PIVOT_CLIMB:
  //       targetState = CLIMB_STATES.NONE;
  //       break;
  //     case RETRACT_CLIMB:
  //       targetState = CLIMB_STATES.PIVOT_CLIMB;
  //       break;
  //     default:
  //       break;
  //   }
  // }
}
