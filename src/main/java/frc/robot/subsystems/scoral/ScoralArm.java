package frc.robot.subsystems.scoral;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SimConstants;
import frc.robot.constants.SubsystemConstants;
import frc.robot.subsystems.commoniolayers.ArmIO;
import frc.robot.subsystems.commoniolayers.ArmIOInputsAutoLogged;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class ScoralArm extends SubsystemBase {
  private final ArmIO scoralArm;
  private final ArmIOInputsAutoLogged saInputs = new ArmIOInputsAutoLogged();

  private final DistanceSensorIO distanceSensor;
  private final DistanceSensorIOInputsAutoLogged dInputs = new DistanceSensorIOInputsAutoLogged();

  private static LoggedTunableNumber kP = new LoggedTunableNumber("CoralScoringArm/kP");
  ;
  private static LoggedTunableNumber kG = new LoggedTunableNumber("CoralScoringArm/kG");
  ;
  private static LoggedTunableNumber kV = new LoggedTunableNumber("CoralScoringArm/kV");

  private static LoggedTunableNumber kA = new LoggedTunableNumber("CoralScoringArm/kA", 0);
  ;
  private static LoggedTunableNumber kS = new LoggedTunableNumber("CoralScoringArm/kS", 0);
  ;
  private static LoggedTunableNumber kI = new LoggedTunableNumber("CoralScoringArm/kI", 0);

  private static double maxVelocityDegPerSec;
  private static double maxAccelerationDegPerSecSquared;

  private TrapezoidProfile armProfile;
  private TrapezoidProfile.Constraints armConstraints;

  private TrapezoidProfile.State armGoalStateDegrees = new TrapezoidProfile.State();
  private TrapezoidProfile.State armCurrentStateDegrees = new TrapezoidProfile.State();

  double goalDegrees;

  private ArmFeedforward armFFModel;

  public static PivotVis measuredVisualizer;
  public static PivotVis setpointVisualizer;

  /** Creates a new Arm. */
  public ScoralArm(ArmIO arm, DistanceSensorIO distanceSensor) {
    this.scoralArm = arm;
    this.distanceSensor = distanceSensor;
    switch (SimConstants.currentMode) {
      case REAL:
        // kG.initDefault(0.32);
        kG.initDefault(0.08);
        // kG.initDefault(0);4
        // kV.initDefault(0.8);
        kV.initDefault(1);
        // kP.initDefault(0.5);
        kP.initDefault(0.7);
        // kP.initDefault(0);
        kA.initDefault(0);
        kS.initDefault(0);
        // kS.initDefault(0);
        kI.initDefault(0);
        break;
      case REPLAY:
        kG.initDefault(0.29);
        kV.initDefault(1);
        kP.initDefault(1.123);
        kA.initDefault(1);
        kS.initDefault(1);
        kI.initDefault(1);
        break;
      case SIM:
        kG.initDefault(0.32);
        kV.initDefault(0.01);
        kP.initDefault(20);
        kA.initDefault(0);
        kS.initDefault(0);
        kI.initDefault(50);
        break;
      default:
        kG.initDefault(0.29);
        kV.initDefault(1);
        kP.initDefault(1.123);
        kA.initDefault(1);
        kS.initDefault(1);
        kI.initDefault(1);
        break;
    }

    measuredVisualizer = new PivotVis("measured", Color.kRed);
    // CHANGE PER ARM
    maxVelocityDegPerSec = 150; // was at 90
    maxAccelerationDegPerSecSquared = 300; // was at 190
    // maxAccelerationDegPerSecSquared = 180;

    armConstraints =
        new TrapezoidProfile.Constraints(maxVelocityDegPerSec, maxAccelerationDegPerSecSquared);
    armProfile = new TrapezoidProfile(armConstraints);
    // setArmGoal(-90);
    // setArmCurrent(getArmPositionDegs());
    armCurrentStateDegrees = armProfile.calculate(0, armCurrentStateDegrees, armGoalStateDegrees);
    armFFModel = new ArmFeedforward(kS.get(), kG.get(), kV.get(), 0);

    measuredVisualizer = new PivotVis("measured pivot scoral", Color.kAzure);
    setpointVisualizer = new PivotVis("setpoint pivot scoral", Color.kGreen);

    updateTunableNumbers();
  }

  public void setBrakeMode(boolean bool) {
    scoralArm.setBrakeMode(bool);
  }

  public double getArmPositionDegs() {
    return saInputs.positionDegs;
  }

  public boolean atGoal(double threshold) {
    return (Math.abs(saInputs.positionDegs - goalDegrees) <= threshold);
  }

  public boolean hasReachedGoal(double goalDegs) {
    Logger.recordOutput(
        "Debug arm has reached goal", Math.abs(saInputs.positionDegs - goalDegs) <= 8);
    return (Math.abs(saInputs.positionDegs - goalDegs) <= 8);
  }

  public boolean shouldWinch() {
    return (Math.abs(saInputs.positionDegs - 29) <= 8);
  }

  private double getArmError() {
    return saInputs.positionSetpointDegs - saInputs.positionDegs;
  }

  public boolean armAtSetpoint(double thresholdDegrees) {
    return (Math.abs(getArmError()) <= thresholdDegrees);
  }

  public void setConstraints(
      double maxVelocityDegreesPerSec, double maxAccelerationDegreesPerSecSquared) {
    armConstraints =
        new TrapezoidProfile.Constraints(
            maxVelocityDegreesPerSec, maxAccelerationDegreesPerSecSquared);
    armProfile = new TrapezoidProfile(armConstraints);
  }

  public void setPositionDegs(double positionDegs, double velocityDegsPerSec) {
    // positionDegs = MathUtil.clamp(positionDegs, 33, 120);
    Logger.recordOutput(
        "Debug Scoral Arm/scoral ffvolts",
        armFFModel.calculate(
            Units.degreesToRadians(positionDegs), Units.degreesToRadians(velocityDegsPerSec)));
    scoralArm.setPositionSetpointDegs(
        positionDegs,
        armFFModel.calculate(
            Units.degreesToRadians(positionDegs), Units.degreesToRadians(velocityDegsPerSec)));
  }

  public void setVolts(double volts) {
    scoralArm.setVoltage(volts);
  }

  public void armStop() {
    scoralArm.stop();
  }

  public void setArmGoal(double goalDegrees) {
    this.goalDegrees = goalDegrees;
    armGoalStateDegrees = new TrapezoidProfile.State(goalDegrees, 0);
  }

  public void setArmCurrent(double currentDegrees) {
    armCurrentStateDegrees = new TrapezoidProfile.State(currentDegrees, 0);
  }

  public double getCANRangeDistance() {
    return dInputs.distanceInches;
  }

  public boolean isCANRangeConnected() {
    return dInputs.connected;
  }

  public double getCANRangeDistanceStd() {
    return dInputs.distanceStd;
  }

  @Override
  public void periodic() {
    scoralArm.updateInputs(saInputs);
    distanceSensor.updateInputs(dInputs);

    measuredVisualizer.update(armCurrentStateDegrees.position);
    armCurrentStateDegrees =
        armProfile.calculate(
            SubsystemConstants.LOOP_PERIOD_SECONDS, armCurrentStateDegrees, armGoalStateDegrees);

    setPositionDegs(armCurrentStateDegrees.position, armCurrentStateDegrees.velocity);

    Logger.processInputs("Scoral Arm", saInputs);
    Logger.processInputs("Scoral Arm/Distance Sensor", dInputs);
    // Logger.recordOutput("Debug Scoral Arm/arm error", getArmError());

    Logger.recordOutput("Debug Scoral Arm/arm goal", goalDegrees);

    // Logger.recordOutput("Debug Scoral Arm/atGoal", atGoal(2));
    // This method will be called once per scheduler run
    measuredVisualizer.update(armCurrentStateDegrees.position);
    setpointVisualizer.update(armGoalStateDegrees.position);

    updateTunableNumbers();
  }

  private void updateTunableNumbers() {
    if (kP.hasChanged(hashCode()) || kI.hasChanged(hashCode())) {
      scoralArm.configurePID(kP.get(), kI.get(), 0);
    }
  }
}
