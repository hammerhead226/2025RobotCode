// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.SubsystemConstants.LED_STATE;
import frc.robot.constants.SubsystemConstants.ScoralArmConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.scoral.ScoralArm;
import frc.robot.subsystems.scoral.ScoralRollers;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeAlgaeFromReef extends SequentialCommandGroup {
  /** Creates a new IntakeAlgaeFromReef. */
  Drive drive;

  ScoralArm scoralArm;
  ScoralRollers scoralRollers;
  Elevator elevator;
  LED led;

  public IntakeAlgaeFromReef(
      Drive drive,
      ScoralArm scoralArm,
      ScoralRollers scoralRollers,
      Elevator elevator,
      LED led,
      double height1,
      double height2) {
    this.drive = drive;
    this.scoralArm = scoralArm;
    this.scoralRollers = scoralRollers;
    this.elevator = elevator;
    this.led = led;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(() -> led.setState(LED_STATE.PURPLE)),
        new ParallelCommandGroup(
            new SetElevatorTarget(elevator, height1, 15),
            new SetScoralArmTarget(scoralArm, ScoralArmConstants.STOW_SETPOINT_DEG, 2),
            new MoveToReefCenter(drive)),
        new SetScoralArmTarget(scoralArm, ScoralArmConstants.STOW_SETPOINT_DEG - 6, 2),
        new WaitUntilCommand(() -> elevator.hasReachedGoal(height1)),
        new InstantCommand(() -> scoralArm.setConstraints(300, 600)),
        new SetScoralArmTarget(scoralArm, 78, 2),
        new InstantCommand(() -> scoralArm.setConstraints(150, 300)),
        scoralRollers.runVoltsCommmand(-2),
        new SetElevatorTarget(elevator, height2, 1.5));
  }

  public IntakeAlgaeFromReef(
      Drive drive,
      ScoralRollers scoralRollers,
      ScoralArm scoralArm,
      Elevator elevator,
      LED led,
      double height1,
      double height2) {
    this.drive = drive;
    this.scoralArm = scoralArm;
    this.scoralRollers = scoralRollers;
    this.elevator = elevator;
    this.led = led;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(() -> led.setState(LED_STATE.PURPLE)),
        new ParallelCommandGroup(
            new SetElevatorTarget(elevator, height1, 15),
            new SetScoralArmTarget(scoralArm, ScoralArmConstants.STOW_SETPOINT_DEG, 2)),
        new SetScoralArmTarget(scoralArm, ScoralArmConstants.STOW_SETPOINT_DEG - 6, 2),
        new WaitUntilCommand(() -> elevator.hasReachedGoal(height1)),
        new InstantCommand(() -> scoralArm.setConstraints(300, 600)),
        new SetScoralArmTarget(scoralArm, 78, 2),
        new InstantCommand(() -> scoralArm.setConstraints(150, 300)),
        scoralRollers.runVoltsCommmand(-2),
        new SetElevatorTarget(elevator, height2, 1.5));
  }
}
