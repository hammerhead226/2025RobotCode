// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
/**
 * Command that on start up gets a command from a supplier and then runs it (effectively a wrapper).
 * Given to triggers so that they do not get locked to one command on inital evalutation.
 *
 * @author Devin Huang
 */
public class ReinitializingCommand extends Command {
  Supplier<Command> commandSupplier;
  Command command;

  /**
   * @param commandSupplier the supplier which returns a command to be run each time this command is
   *     reinitalized
   * @param requirements optional parameters to specify any number of subsystems required
   */
  public ReinitializingCommand(Supplier<Command> commandSupplier, Subsystem... requirements) {
    this.commandSupplier = commandSupplier;
    addRequirements(requirements);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.command = commandSupplier.get(); // is it this. or no this.
    command.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    command.execute();
  }

  // Called once the command ends or is interrupted.
  /*
   * on end, the ReinitializingCommand also completely cancels the command its running
   */
  @Override
  public void end(boolean interrupted) {
    command.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return command.isFinished();
  }
}
