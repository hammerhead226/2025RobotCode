// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotMap.ledIDs;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.newalgaeintake.AlgaeIntakeArm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbCommands extends Command {
  private final AlgaeIntakeArm climberArm;
  private final LED led;
  private final Drive drive;
  /** Creates a new ClimbCommands. */
  public ClimbCommands(AlgaeIntakeArm climberArm, Drive drive, LED led) {
    this.climberArm = climberArm;
    this.led = led;
    this.drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climberArm, drive, led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
