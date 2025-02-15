// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants.ReefHeight;
import frc.robot.constants.SubsystemConstants.AlgaeState;
import frc.robot.constants.SubsystemConstants.CoralScorerConstants.AlgaeScorerFlywheelConstants;
import frc.robot.constants.SubsystemConstants.CoralScorerConstants.CoralScorerArmConstants;
import frc.robot.constants.SubsystemConstants.ElevatorConstants;
import frc.robot.subsystems.coralscorer.CoralScorerArm;
import frc.robot.subsystems.coralscorer.CoralScorerFlywheel;
import frc.robot.subsystems.elevator.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakingAlgae extends Command {
  private final Elevator elevator;
  private final CoralScorerFlywheel algaeIntake;
  private final CoralScorerArm arm;
  private final ReefHeight reefHeight;

  /** Creates a new IntakingAlgae. */
  public IntakingAlgae(
      Elevator elevator,
      CoralScorerFlywheel algaeIntake,
      CoralScorerArm arm,
      ReefHeight reefHeight) {
    this.elevator = elevator;
    this.algaeIntake = algaeIntake;
    this.arm = arm;
    this.reefHeight = reefHeight;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator, algaeIntake, arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setElevatorTarget(reefHeight.height, ElevatorConstants.DEFAULT_THRESHOLD);
    arm.setPositionDegs(
        CoralScorerArmConstants.INTAKE_SETPOINT_DEG,
        CoralScorerArmConstants.ARM_VELOCITY_DEGPERSEC);
    algaeIntake.runVelocity(AlgaeScorerFlywheelConstants.FLYWHEEL_VELOCITY_DEGPERSEC);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  led.setState(LED_STATE.FLASHING_GREEN);
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algaeIntake.flywheelStop();
    arm.setPositionDegs(
        CoralScorerArmConstants.STOW_SETPOINT_DEG, CoralScorerArmConstants.ARM_VELOCITY_DEGPERSEC);
    elevator.setElevatorTarget(0, ElevatorConstants.DEFAULT_THRESHOLD);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return algaeIntake.seesAlgae() == AlgaeState.CURRENT;
  }
}
