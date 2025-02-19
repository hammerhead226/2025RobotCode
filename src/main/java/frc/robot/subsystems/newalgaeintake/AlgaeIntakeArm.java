// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.newalgaeintake;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SimConstants;
import frc.robot.constants.SubsystemConstants;
import frc.robot.subsystems.coralscorer.ClimberVis;
import frc.robot.subsystems.coralscorer.PivotVis;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class AlgaeIntakeArm extends SubsystemBase {
  private final AlgaeIntakeArmIO arm;
  private final AlgaeIntakeArmIOInputsAutoLogged pInputs = new AlgaeIntakeArmIOInputsAutoLogged();

  private static LoggedTunableNumber kP = new LoggedTunableNumber("algaeArmkP");
  private static LoggedTunableNumber kG = new LoggedTunableNumber("algaeArmkG");
  private static LoggedTunableNumber kV = new LoggedTunableNumber("algaeArmkV");

  private static double maxVelocityDegPerSec;
  private static double maxAccelerationDegPerSecSquared;

  private TrapezoidProfile armProfile;
  private TrapezoidProfile.Constraints armConstraints;

  private TrapezoidProfile.State armGoalStateDegrees = new TrapezoidProfile.State();
  private TrapezoidProfile.State armCurrentStateDegrees = new TrapezoidProfile.State();

  double goalDegrees;

  private ArmFeedforward armFFModel;
  private final PivotVis measuredVisualizer;
  private final PivotVis setpointVisualizer;
  private final ClimberVis climberMeasuredVisualizer;
  private final ClimberVis climberSetpointVisualizer;

  /** Creates a new Arm. */
  public AlgaeIntakeArm(AlgaeIntakeArmIO arm) {
    this.arm = arm;
    switch (SimConstants.currentMode) {
      case REAL:
        kG.initDefault(0.29);
        kV.initDefault(1);
        kP.initDefault(1.123);
        break;
      case REPLAY:
        kG.initDefault(0.29);
        kV.initDefault(1);
        kP.initDefault(1.123);
        break;
      case SIM:
        kG.initDefault(0.29);
        kV.initDefault(1);
        kP.initDefault(1.123);
        break;
      default:
        kG.initDefault(0.29);
        kV.initDefault(1);
        kP.initDefault(1.123);
        break;
    }

    // CHANGE PER ARM
    maxVelocityDegPerSec = 500;
    maxAccelerationDegPerSecSquared = 500;
    // maxAccelerationDegPerSecSquared = 180;

    armConstraints =
        new TrapezoidProfile.Constraints(maxVelocityDegPerSec, maxAccelerationDegPerSecSquared);
    armProfile = new TrapezoidProfile(armConstraints);

    setArmGoal(20);
    // setArmCurrent(getArmPositionDegs());
    armCurrentStateDegrees = armProfile.calculate(0, armCurrentStateDegrees, armGoalStateDegrees);
    measuredVisualizer = new PivotVis("intake vis ", Color.kBrown);
    setpointVisualizer = new PivotVis("intake setpoint vis", Color.kGreen);
    climberMeasuredVisualizer = new ClimberVis("climber measured vis", Color.kOrange);
    climberSetpointVisualizer = new ClimberVis("climber setpoint vis", Color.kGreen);
    updateTunableNumbers();
  }

  public void setBrakeMode(boolean bool) {
    arm.setBrakeMode(bool);
  }

  public double getArmPositionDegs() {
    return pInputs.positionDegs;
  }

  public boolean atGoal(double threshold) {
    return (Math.abs(pInputs.positionDegs - goalDegrees) <= threshold);
  }

  private double getArmError() {
    return pInputs.positionSetpointDegs - pInputs.positionDegs;
  }

  public void setPositionDegs(double positionDegs, double velocityDegsPerSec) {
    // positionDegs = MathUtil.clamp(positionDegs, 33, 120);
    arm.setPositionSetpointDegs(
        positionDegs, armFFModel.calculate(positionDegs, velocityDegsPerSec));
  }

  public void armStop() {
    arm.stop();
  }

  public void setArmGoal(double goalDegrees) {
    this.goalDegrees = goalDegrees;
    armGoalStateDegrees = new TrapezoidProfile.State(goalDegrees, 0);
  }

  public void setArmCurrent(double currentDegrees) {
    armCurrentStateDegrees = new TrapezoidProfile.State(currentDegrees, 0);
  }

  public Command setArmTarget(double goalDegrees, double thresholdDegrees) {

    return new InstantCommand(() -> setArmGoal(goalDegrees), this)
        .until(() -> atGoal(thresholdDegrees));
  }

  @Override
  public void periodic() {
    arm.updateInputs(pInputs);

    armCurrentStateDegrees =
        armProfile.calculate(
            SubsystemConstants.LOOP_PERIOD_SECONDS, armCurrentStateDegrees, armGoalStateDegrees);

    setPositionDegs(armCurrentStateDegrees.position, armCurrentStateDegrees.velocity);

    Logger.processInputs("Arm", pInputs);
    Logger.recordOutput("arm error", getArmError());

    Logger.recordOutput("arm goal", goalDegrees);
    // This method will be called once per scheduler run
    measuredVisualizer.update(armCurrentStateDegrees.position);
    climberMeasuredVisualizer.update(armCurrentStateDegrees.position);
    setpointVisualizer.update(armGoalStateDegrees.position);
    climberSetpointVisualizer.update(armGoalStateDegrees.position);
    measuredVisualizer.updateLength(0.8);
    setpointVisualizer.updateLength(0.8);
    updateTunableNumbers();
  }

  private void updateTunableNumbers() {
    if (kP.hasChanged(hashCode())) {
      arm.configurePID(kP.get(), 0, 0);
    }
    if (kG.hasChanged(hashCode()) || kV.hasChanged(hashCode())) {
      armFFModel = new ArmFeedforward(0, kG.get(), kV.get(), 0);
    }
  }
}
