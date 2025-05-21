// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.SubsystemConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.scoral.ScoralArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

/**
This sequential command group is supposed to extend the elevator 
to the required position to climb the barge and then set the 
scoral arm to the required position to initiate the climb
*/

public class BargeExtend extends SequentialCommandGroup {
  public BargeExtend(Elevator elevator, ScoralArm scoralArm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    addCommands(
        new SetElevatorTarget(elevator, SubsystemConstants.ElevatorConstants.BARGE_SETPOINT, 15),
        new WaitUntilCommand(() -> elevator.atGoal(15)),
        new SetScoralArmTarget(
            scoralArm, SubsystemConstants.ScoralArmConstants.BARGE_BACK_SETPOINT_DEG, 2));
  }
}
