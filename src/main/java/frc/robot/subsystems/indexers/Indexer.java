// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexers;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SimConstants;
import frc.robot.constants.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */
  private final IndexerIO indexer;

  private final IndexerIOInputsAutoLogged iInputs = new IndexerIOInputsAutoLogged();

  private static double kP;
  private static double kG;
  private static double kV;

  private static double maxVelocityRotPerSec;
  private static double maxAccelerationRotPerSecSquared;

  private TrapezoidProfile indexerProfile;
  private TrapezoidProfile.Constraints indexerConstraints;

  private TrapezoidProfile.State indexerGoalStateRotations = new TrapezoidProfile.State();
  private TrapezoidProfile.State indexerCurrentStateRotations = new TrapezoidProfile.State();

  private ElevatorFeedforward ff;

  public Indexer(IndexerIO indexer) {
    this.indexer = indexer;

    switch (SimConstants.currentMode) {
      case REAL:
        kG = 0.0;
        kV = 0.0;
        kP = 0.0;
        break;
      case REPLAY:
        kG = 0.0;
        kV = 0.0;
        kP = 0.0;
        break;
      case SIM:
        kG = 0.0;
        kV = 0.0;
        kP = 0.0;
        break;
      default:
        kG = 0.0;
        kV = 0.0;
        kP = 0.0;
        break;
    }

    // CHANGE THESE VALUES TO MATCH THE INDEXER
    maxVelocityRotPerSec = 1;
    maxAccelerationRotPerSecSquared = 1;

    indexerConstraints =
        new TrapezoidProfile.Constraints(maxVelocityRotPerSec, maxAccelerationRotPerSecSquared);
    indexerProfile = new TrapezoidProfile(indexerConstraints);

    indexerCurrentStateRotations =
        indexerProfile.calculate(0, indexerCurrentStateRotations, indexerGoalStateRotations);

    indexer.configurePID(kP, 0, 0);
    ff = new ElevatorFeedforward(0, kG, kV);
  }

  public boolean indexerAtGoal(double thersholdInches) {

    return (Math.abs(indexerCurrentStateRotations.position - indexerGoalStateRotations.position)
        <= thersholdInches);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    indexer.updateInputs(iInputs);

    indexerCurrentStateRotations =
        indexerProfile.calculate(
            SubsystemConstants.LOOP_PERIOD_SECONDS,
            indexerCurrentStateRotations,
            indexerGoalStateRotations);

    indexer.setPositionSetpoint(
        indexerCurrentStateRotations.position * 360,
        ff.calculate(
                LinearVelocity.ofBaseUnits(indexerCurrentStateRotations.velocity, InchesPerSecond))
            .in(Volts));

    Logger.processInputs("Indexer", iInputs);
  }

  public void index(double linearDistanceInches) {

    double rollerDiameterInches = 1;
    indexerGoalStateRotations.position += linearDistanceInches / (rollerDiameterInches * Math.PI);
  }

  public Command indexCommand(double linearDistanceInches, double thersholdInches) {

    return new InstantCommand(() -> index(linearDistanceInches), this)
        .until(() -> indexerAtGoal(thersholdInches));
  }
}
