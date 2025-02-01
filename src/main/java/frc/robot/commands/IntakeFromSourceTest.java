// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SubsystemConstants.CoralScorerConstants.CoralScorerArmConstants;
import frc.robot.constants.SubsystemConstants.CoralScorerConstants.CoralScorerFlywheelConstants;
import frc.robot.constants.SubsystemConstants.CoralState;
import frc.robot.constants.SubsystemConstants.ElevatorConstants;
import frc.robot.subsystems.coralscorer.CoralScorerArm;
import frc.robot.subsystems.coralscorer.CoralScorerFlywheel;
import frc.robot.subsystems.elevator.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeFromSourceTest extends Command {
  /** Creates a new IntakeFromSource. */
  private final CoralScorerFlywheel coralIntake;

  private final CoralScorerArm arm;
  private final Elevator elevator;

  public IntakeFromSourceTest(
      CoralScorerFlywheel coralIntake, CoralScorerArm arm, Elevator elevator) {
    this.coralIntake = coralIntake;
    this.elevator = elevator;
    this.arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coralIntake, elevator, arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setElevatorTarget(10, ElevatorConstants.DEFAULT_THRESHOLD);
    arm.setArmTarget(70, 2);
    coralIntake.runVelocity(CoralScorerFlywheelConstants.FLYWHEEL_VELOCITY_DEGPERSEC);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralIntake.flywheelStop();
    arm.setPositionDegs(
        CoralScorerArmConstants.STOW_SETPOINT_DEG, CoralScorerArmConstants.ARM_VELOCITY_DEGPERSEC);
    elevator.setElevatorTarget(1, ElevatorConstants.DEFAULT_THRESHOLD);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return coralIntake.seesCoral() == CoralState.SENSOR
        || coralIntake.seesCoral() == CoralState.CURRENT;
  }
}
