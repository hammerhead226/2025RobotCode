// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.constants.SubsystemConstants.ElevatorConstants;
import frc.robot.constants.SubsystemConstants.ScoralArmConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.scoral.ScoralArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

/**
 * This ParallelCommandGroup is meant to move the elevator and the scoral
 * to the required setpoints for depositing an algaue in the processor
 * at the same time.
 */

public class MoveToProcessorSetpoints extends ParallelCommandGroup {
  private final ScoralArm scoralArm;
  private final Elevator elevator;

  public MoveToProcessorSetpoints(ScoralArm scoralArm, Elevator elevator) {
    this.elevator = elevator;
    this.scoralArm = scoralArm;

    addCommands(
        new SetElevatorTarget(elevator, ElevatorConstants.PROCESSOR_SETPOINT_INCHES, 2),
        new SetScoralArmTarget(scoralArm, ScoralArmConstants.PROCESSOR_SETPOINT_DEG, 2));
  }
}
