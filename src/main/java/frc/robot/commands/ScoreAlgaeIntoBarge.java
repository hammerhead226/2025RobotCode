// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.SubsystemConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.scoral.ScoralArm;
import frc.robot.subsystems.scoral.ScoralRollers;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreAlgaeIntoBarge extends SequentialCommandGroup {
  /** Creates a new ScoreAlgaeIntoBarge. */
  private final ScoralArm scoralArm;

  private final ScoralRollers scoralRollers;
  private final Elevator elevator;

  public ScoreAlgaeIntoBarge(
      Elevator m_elevator, ScoralArm m_scoralArm, ScoralRollers m_scoralRollers) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.elevator = m_elevator;
    this.scoralArm = m_scoralArm;
    this.scoralRollers = m_scoralRollers;

    addCommands(
        new ParallelCommandGroup(
            new SetElevatorTarget(elevator, SubsystemConstants.ElevatorConstants.BARGE_SETPOINT, 1),
            new SetScoralArmTarget(
                scoralArm, SubsystemConstants.ScoralArmConstants.BARGE_BACK_SETPOINT_DEG, 2)));
  }
}
