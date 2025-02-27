// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.SubsystemConstants;
import frc.robot.constants.SubsystemConstants.AlgaeState;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.scoral.ScoralArm;
import frc.robot.subsystems.scoral.ScoralRollers;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeAlgae extends SequentialCommandGroup {
  /** Creates a new IntakeAlgae. */
  private final ScoralArm scoralArm;

  private final ScoralRollers scoralRollers;
  private final Elevator elevator;

  public IntakeAlgae(
      Elevator m_elevator, ScoralArm m_scoralArm, ScoralRollers m_scoralRollers, double height) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    this.elevator = m_elevator;
    this.scoralArm = m_scoralArm;
    this.scoralRollers = m_scoralRollers;

    addCommands(
        new SequentialCommandGroup(
            elevator.setElevatorTarget(height, 3),
            scoralArm.setArmTarget(30, 2),
            new InstantCommand(() -> scoralRollers.runVolts(2)),
            new WaitUntilCommand(() -> scoralRollers.seesAlgae() == AlgaeState.CURRENT),
            scoralRollers.stopCommand(),
            new ToReefHeight(
                m_elevator,
                m_scoralArm,
                SubsystemConstants.ElevatorConstants.L2_SETPOINT_INCHES,
                SubsystemConstants.ScoralArmConstants.LOW_CORAL_SCORING_SETPOINT_DEG)));
  }
}
