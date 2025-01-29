// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.FieldConstants.ReefHeight;
import frc.robot.subsystems.coralscorer.CoralScorerArm;
import frc.robot.subsystems.elevator.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetScoringHeights extends SequentialCommandGroup {
  /** Creates a new SetArmAngleElevatorHeight. */
  double offsetInches = 0;
  public SetScoringHeights(ReefHeight height, Elevator elevator, CoralScorerArm csArm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // TODO:: CHANGE THE VALUES LATER
    addCommands(
        csArm.setArmTarget(height == ReefHeight.L4 ? 90 : 60, 3),
        new WaitUntilCommand(() -> csArm.atGoal(10)),
        elevator.setElevatorTarget(height.height + offsetInches, 3));
  }
}
