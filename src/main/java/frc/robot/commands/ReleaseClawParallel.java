package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.constants.FieldConstants.ReefHeight;
import frc.robot.subsystems.coralscorer.CoralScorerArm;
import frc.robot.subsystems.coralscorer.CoralScorerFlywheel;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.led.LED;

public class ReleaseClawParallel extends ParallelCommandGroup {
  private final Elevator elevator;
  private final CoralScorerArm pivot;
  private final CoralScorerFlywheel flywheel;
  private final ReefHeight scoringLevel;

  public ReleaseClawParallel(
      ReefHeight scoringLevel,
      Elevator elevator,
      CoralScorerArm pivot,
      CoralScorerFlywheel flywheel,
      LED led) {
    this.pivot = pivot;
    this.elevator = elevator;
    this.flywheel = flywheel;
    this.scoringLevel = scoringLevel;
    addCommands(
        elevator.setElevatorTarget(scoringLevel.height, 0.5),
        pivot.setArmTarget(scoringLevel.pitch, 1),
        flywheel.runVoltsCommmand(12));
  }
}
