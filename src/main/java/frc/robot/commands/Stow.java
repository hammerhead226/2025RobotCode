// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SubsystemConstants.LED_STATE;
import frc.robot.subsystems.coralscorer.CoralScorerArm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.led.LED;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Stow extends Command {
  private final CoralScorerArm arm;
  private final Elevator elevator;
  private double elevatorpos;
  private double armangle;
  private final LED led;

  /** Creates a new Stow. */
  public Stow(CoralScorerArm arm, Elevator elevator, LED led) {

    this.arm = arm;
    this.elevator = elevator;
    this.led = led;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setPositionExtend(5, 20);
    arm.setPositionDegs(0, 20);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorpos = elevator.getElevatorPosition();
    armangle = arm.getArmPositionDegs();
    led.setState(LED_STATE.GREEN);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(elevatorpos - 10) >= 2 && Math.abs(armangle - 10) >= 2;
  }
}
