package frc.robot.commands.algaeintosource;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SubsystemConstants;
import frc.robot.subsystems.coralscorer.CoralScorerArm;
import frc.robot.subsystems.elevator.Elevator;

public class ReadyForAlgaeScore extends Command {
  Elevator elevator;
  CoralScorerArm coralScorerArm;

  public ReadyForAlgaeScore(Elevator elevator, CoralScorerArm coralScorerArm) {
    this.elevator = elevator;
    this.coralScorerArm = coralScorerArm;

    addRequirements(elevator, coralScorerArm);
  }

  @Override
  public void initialize() {
    elevator.setExtenderGoal(SubsystemConstants.ElevatorConstants.SCORING_SETPOINT_POS);
    coralScorerArm.setArmGoal(
        SubsystemConstants.CoralScorerConstants.CoralScorerArmConstants.SCORING_SETPOINT_DEG);
  }

  @Override
  public boolean isFinished() {
    return elevator.elevatorAtSetpoint(SubsystemConstants.ElevatorConstants.DEFAULT_THRESHOLD)
        && coralScorerArm.atGoal(
            SubsystemConstants.CoralScorerConstants.CoralScorerArmConstants.DEFAULT_THRESHOLD);
  }
}
