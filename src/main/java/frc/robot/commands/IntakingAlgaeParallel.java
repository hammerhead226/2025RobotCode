// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.constants.SubsystemConstants.ElevatorConstants;
import frc.robot.subsystems.coralscorer.CoralScorerArm;
import frc.robot.subsystems.coralscorer.CoralScorerFlywheel;
import frc.robot.subsystems.elevator.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakingAlgaeParallel extends ParallelCommandGroup {
  private final Elevator elevator;
  private final CoralScorerArm arm;
  private final CoralScorerFlywheel algaeIntake;
  /** Creates a new IntakingAlgaeParallel. */
  public IntakingAlgaeParallel(
      Elevator elevator, CoralScorerArm arm, CoralScorerFlywheel algaeIntake) {
    this.elevator = elevator;
    this.arm = arm;
    this.algaeIntake = algaeIntake;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        elevator.setElevatorTarget(1, ElevatorConstants.DEFAULT_THRESHOLD),
        arm.setArmTarget(70, 2),
        new InstantCommand(() -> algaeIntake.runVolts(5)));
  }
}
