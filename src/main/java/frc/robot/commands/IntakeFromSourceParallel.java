// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.coralscorer.CoralScorerArm;
import frc.robot.subsystems.coralscorer.CoralScorerFlywheel;
import frc.robot.subsystems.elevator.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeFromSourceParallel extends ParallelCommandGroup {
  private final CoralScorerFlywheel coralIntake;
  private final CoralScorerArm arm;
  private final Elevator elevator;

  /** Creates a new IntakeFromSourceTestTwo. */
  public IntakeFromSourceParallel(
      CoralScorerFlywheel coralIntake, CoralScorerArm arm, Elevator elevator) {
    this.coralIntake = coralIntake;
    this.elevator = elevator;
    this.arm = arm;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        elevator.setElevatorTarget(0.75, 0.01),
        arm.setArmTarget(32, 2),
        new InstantCommand(() -> coralIntake.runVolts(5)));
  }
}
