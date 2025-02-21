package frc.robot.commands.algaeintosource;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.coralscorer.CoralScorerArm;
import frc.robot.subsystems.coralscorer.CoralScorerFlywheel;
import frc.robot.subsystems.elevator.Elevator;

public class AlgaeIntoSource extends Command {
  Elevator elevator;
  CoralScorerArm coralScorerArm;
  CoralScorerFlywheel coralScorerFlywheel;

  Command sequence;

  public AlgaeIntoSource(
      Elevator elevator, CoralScorerArm coralScorerArm, CoralScorerFlywheel coralScorerFlywheel) {
    this.elevator = elevator;
    this.coralScorerArm = coralScorerArm;
    this.coralScorerFlywheel = coralScorerFlywheel;

    addRequirements(elevator, coralScorerArm, coralScorerFlywheel);
    sequence =
        new SequentialCommandGroup(
            new ReadyForAlgaeScore(elevator, coralScorerArm),
            new ReleaseAlgae(coralScorerFlywheel));
  }

  @Override
  public void initialize() {
    sequence.initialize();
  }

  @Override
  public void execute() {
    sequence.execute();
  }

  @Override
  public void end(boolean interrupted) {
    sequence.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return sequence.isFinished();
  }
}
