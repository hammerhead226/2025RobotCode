// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SimConstants;
import frc.robot.constants.SubsystemConstants;
import frc.robot.subsystems.scoral.ClimberVis;
import frc.robot.subsystems.scoral.PivotVis;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class ClimberArm extends SubsystemBase {
  private final ClimberArmIO arm;
  private final ClimberArmIOInputsAutoLogged pInputs = new ClimberArmIOInputsAutoLogged();

  private static LoggedTunableNumber kP = new LoggedTunableNumber("Climber/kP");
  private static LoggedTunableNumber kG = new LoggedTunableNumber("Climber/kG");
  private static LoggedTunableNumber kV = new LoggedTunableNumber("Climber/kV");

  private static double maxVelocityDegPerSec;
  private static double maxAccelerationDegPerSecSquared;

  private boolean closedLoop = true;

  private TrapezoidProfile armProfile;
  private TrapezoidProfile.Constraints armConstraints;

  private TrapezoidProfile.State armGoalStateDegrees = new TrapezoidProfile.State();
  private TrapezoidProfile.State armCurrentStateDegrees = new TrapezoidProfile.State();

  double goalDegrees;

  private final ArmFeedforward armFFModel;
  private final PivotVis measuredVisualizer;
  private final PivotVis setpointVisualizer;
  private final ClimberVis climberMeasuredVisualizer;
  private final ClimberVis climberSetpointVisualizer;

  /** Creates a new Arm. */
  public ClimberArm(ClimberArmIO arm) {
    this.arm = arm;
    switch (SimConstants.currentMode) {
      case REAL:
        // RETUNE
        kG.initDefault(0.0);
        kV.initDefault(0.6);
        kP.initDefault(0.8);
        break;
      case REPLAY:
        kG.initDefault(0.29);
        kV.initDefault(1);
        kP.initDefault(1.123);
        break;
      case SIM:
        kG.initDefault(0.29);
        kV.initDefault(1);
        kP.initDefault(20);
        break;
      default:
        kG.initDefault(0.29);
        kV.initDefault(1);
        kP.initDefault(1.123);
        break;
    }

    armFFModel = new ArmFeedforward(0, 0.01, 0.015);

    // CHANGE PER ARM
    maxVelocityDegPerSec = 60;
    maxAccelerationDegPerSecSquared = 100;
    // maxAccelerationDegPerSecSquared = 180;

    armConstraints =
        new TrapezoidProfile.Constraints(maxVelocityDegPerSec, maxAccelerationDegPerSecSquared);
    armProfile = new TrapezoidProfile(armConstraints);

    // setArmGoal(90);
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

  public boolean hasReachedGoal(double goalDegs) {
    return (Math.abs(pInputs.positionDegs - goalDegs) <= 2);
  }

  public void setPositionDegs(double positionDegs, double velocityDegsPerSec) {
    closedLoop = true;
    // positionDegs = MathUtil.clamp(positionDegs, 33, 120);
    arm.setPositionSetpointDegs(
        positionDegs,
        armFFModel.calculate(
            Units.degreesToRadians(positionDegs), Units.degreesToRadians(velocityDegsPerSec)));
  }

  public boolean isAt(double positionDegs, double threshold) {
    return Math.abs(pInputs.positionDegs - positionDegs) <= threshold;
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

  public Command zero() {
    return new InstantCommand(() -> arm.zeroPosition(), this);
  }

  public void setVoltage(double volts) {
    closedLoop = false;
    arm.setVoltage(volts);
  }

  public Command runVoltsCommand(double volts) {
    return new InstantCommand(() -> setVoltage(volts), this);
  }

  @Override
  public void periodic() {
    arm.updateInputs(pInputs);

    armCurrentStateDegrees =
        armProfile.calculate(
            SubsystemConstants.LOOP_PERIOD_SECONDS, armCurrentStateDegrees, armGoalStateDegrees);

    // if (closedLoop) {
    setPositionDegs(armCurrentStateDegrees.position, armCurrentStateDegrees.velocity);
    // }

    Logger.processInputs("Climber Arm", pInputs);
    // Logger.recordOutput("Debug Climb Arm/arm error", getArmError());

    Logger.recordOutput("Debug Climb Arm/arm goal", goalDegrees);
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
  }
}
