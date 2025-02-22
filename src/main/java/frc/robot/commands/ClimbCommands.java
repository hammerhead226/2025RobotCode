// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SubsystemConstants.LED_STATE;
import frc.robot.subsystems.climber.ClimberArm;
import frc.robot.subsystems.led.LED;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbCommands extends Command {
  private final LED led;
  private final ClimberArm climb;
  private final double goal;
  /** Creates a new ClimbCommands. */
  public ClimbCommands(LED led, ClimberArm climb, double goal) {
    this.led = led;
    this.climb = climb;
    this.goal = goal;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climb.setPositionDegs(goal, 90);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    led.setState(LED_STATE.FLASHING_GREY);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(climb.getArmPositionDegs() - goal) <= 5;
  }
}
