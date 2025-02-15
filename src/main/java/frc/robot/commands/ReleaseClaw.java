package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants.ReefHeight;
import frc.robot.constants.SubsystemConstants.LED_STATE;
import frc.robot.subsystems.coralscorer.CoralScorerArm;
import frc.robot.subsystems.coralscorer.CoralScorerFlywheel;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.led.LED;

public class ReleaseClaw extends Command {
  private final Elevator elevator;
  private final CoralScorerArm pivot;
  private final CoralScorerFlywheel flywheel;
  private final ReefHeight scoringLevel;
  private final LED led;

  public ReleaseClaw(
      ReefHeight scoringLevel,
      Elevator elevator,
      CoralScorerArm pivot,
      CoralScorerFlywheel flywheel,
      LED led) {
    this.pivot = pivot;
    this.elevator = elevator;
    this.flywheel = flywheel;
    this.scoringLevel = scoringLevel;
    this.led = led;
    addRequirements(elevator, pivot, led);
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
