// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.scoral.ScoralArm;
import frc.robot.subsystems.scoral.ScoralRollers;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreCoral extends SequentialCommandGroup {
  /** Creates a new scoringCoral. */
  private final ScoralRollers scoralRollers;

  private final Elevator elevator;
  private final ScoralArm scoralArm;

  public ScoreCoral(Elevator elevator, ScoralArm scoralArm, ScoralRollers scoralRollers) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    this.scoralRollers = scoralRollers;
    this.elevator = elevator;
    this.scoralArm = scoralArm;
    addCommands(scoralRollers.runVoltsCommmand(2.6), new WaitCommand(0.5));
  }
}
