package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants.ReefHeight;
import frc.robot.subsystems.coralscorer.CoralScorerArm;
import frc.robot.subsystems.coralscorer.CoralScorerFlywheel;
import frc.robot.subsystems.elevator.Elevator;

public class ReleaseClaw extends Command {
  private final Elevator elevator;
  private final CoralScorerArm pivot;
  private final CoralScorerFlywheel flywheel;
  private final ReefHeight scoringLevel;

  public ReleaseClaw(
      ReefHeight scoringLevel,
      Elevator elevator,
      CoralScorerArm pivot,
      CoralScorerFlywheel flywheel) {
    this.pivot = pivot;
    this.elevator = elevator;
    this.flywheel = flywheel;
    this.scoringLevel = scoringLevel;
    addRequirements(elevator, pivot);
  }

  @Override
  public void initialize() {
    elevator.setExtenderGoal(scoringLevel.height);
    pivot.setArmGoal(scoringLevel.pitch);
    flywheel.runVolts(12);
    end(true);
  }

  @Override
  public void execute() {
    led.setState(LED_STATE.FLASHING_ORANGE);

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
