// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.scoral.ScoralArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ToReefHeight extends SequentialCommandGroup {
  /** Creates a new goToReefHeight. */
  private final ScoralArm scoralArm;

  private final Elevator elevator;

  public ToReefHeight(
      Elevator m_elevator, ScoralArm m_scoralArm, double heightInch, double pitchDegs) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.elevator = m_elevator;
    this.scoralArm = m_scoralArm;

    addCommands(
        new SequentialCommandGroup(
            new SetElevatorTarget(elevator, heightInch, 15),
            new SetScoralArmTarget(scoralArm, pitchDegs, 3.5)));
    // new WaitUntilCommand(() -> elevator.atGoal(2)),
    // scoralArm.setArmTarget(pitchDegs, 2)));
    // new SetScoralArmTarget(pitchDegs, 2, m_scoralArm)));

    //
  }
}
