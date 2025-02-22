// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.constants.SubsystemConstants;
import frc.robot.constants.SubsystemConstants.LED_STATE;
import frc.robot.subsystems.coralscorer.CoralScorerArm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.led.LED;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Stow extends ParallelCommandGroup {
  /** Creates a new Stow. */
  private Elevator elevator;

  private CoralScorerArm arm;
  // private LED led;

  public Stow(Elevator elevator, CoralScorerArm arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.elevator = elevator;
    this.arm = arm;
    // this.led = led;

    addCommands(
        elevator.setElevatorTarget(0.0, SubsystemConstants.ElevatorConstants.DEFAULT_THRESHOLD),
        arm.setArmTarget(0.0, 2.0), // TODO: put this in constants
        // led.setState(LED_STATE.FLASHING_BLUE)
        );
  }
}
