// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexers;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SimConstants;
import frc.robot.constants.SubsystemConstants;
import frc.robot.subsystems.commoniolayers.IndexerIO;
import frc.robot.subsystems.commoniolayers.IndexerIOInputsAutoLogged;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */
  private final IndexerIO indexer;

  private final IndexerIOInputsAutoLogged iInputs = new IndexerIOInputsAutoLogged();

  private static LoggedTunableNumber kP;
  private static LoggedTunableNumber kG;
  private static LoggedTunableNumber kV;

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
        kG.initDefault(0.0);
        kV.initDefault(0.0);
        kP.initDefault(0.0);
        break;
      case REPLAY:
        kG.initDefault(0.0);
        kV.initDefault(0.0);
        kP.initDefault(0.0);
        break;
      case SIM:
        kG.initDefault(0.0);
        kV.initDefault(0.0);
        kP.initDefault(0.0);
        break;
      default:
        kG.initDefault(0.0);
        kV.initDefault(0.0);
        kP.initDefault(0.0);
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

    updateTunableNumbers();
  }

  public boolean indexerAtGoal(double thresholdInches) {

    return (Math.abs(indexerCurrentStateRotations.position - indexerGoalStateRotations.position)
        <= thresholdInches);
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
        ff.calculate(indexerCurrentStateRotations.velocity));

    Logger.processInputs("Indexer", iInputs);

    updateTunableNumbers();
  }

  private void updateTunableNumbers() {
    if (kP.hasChanged(hashCode())) {
      indexer.configurePID(kP.get(), 0, 0);
    }
    if (kG.hasChanged(hashCode()) || kV.hasChanged(hashCode())) {
      ff = new ElevatorFeedforward(0, kG.get(), kV.get());
    }
  }

  public void index(double linearDistanceInches) {

    double rollerDiameterInches = 1;
    indexerGoalStateRotations.position += linearDistanceInches / (rollerDiameterInches * Math.PI);
  }

  public Command indexCommand(double linearDistanceInches, double thresholdInches) {

    return new InstantCommand(() -> index(linearDistanceInches), this)
        .until(() -> indexerAtGoal(thresholdInches));
  }
}
