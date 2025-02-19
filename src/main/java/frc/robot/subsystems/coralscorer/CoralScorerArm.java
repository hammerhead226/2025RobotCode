package frc.robot.subsystems.coralscorer;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SimConstants;
import frc.robot.constants.SubsystemConstants;
import frc.robot.subsystems.commoniolayers.ArmIO;
import frc.robot.subsystems.commoniolayers.ArmIOInputsAutoLogged;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class CoralScorerArm extends SubsystemBase {
  private final ArmIO coralScorerArm;
  private final ArmIOInputsAutoLogged csaInputs = new ArmIOInputsAutoLogged();

  private static LoggedTunableNumber kP = new LoggedTunableNumber("CoralScoringArm/kP");
  ;
  private static LoggedTunableNumber kG = new LoggedTunableNumber("CoralScoringArm/kG");
  ;
  private static LoggedTunableNumber kV = new LoggedTunableNumber("CoralScoringArm/kV");
  ;

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
  public CoralScorerArm(ArmIO arm) {
    this.coralScorerArm = arm;
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
        kG.initDefault(0);
        kV.initDefault(1);
        kP.initDefault(1);
        break;
      default:
        kG.initDefault(0.29);
        kV.initDefault(1);
        kP.initDefault(1.123);
        break;
    }

    measuredVisualizer = new PivotVis("measured", Color.kRed);
    // CHANGE PER ARM
    maxVelocityDegPerSec = 90;
    maxAccelerationDegPerSecSquared = 180;
    // maxAccelerationDegPerSecSquared = 180;

    armConstraints =
        new TrapezoidProfile.Constraints(maxVelocityDegPerSec, maxAccelerationDegPerSecSquared);
    armProfile = new TrapezoidProfile(armConstraints);

    setArmGoal(-90);
    // setArmCurrent(getArmPositionDegs());
    armCurrentStateDegrees = armProfile.calculate(0, armCurrentStateDegrees, armGoalStateDegrees);
    armFFModel = new ArmFeedforward(0, kG.get(), kV.get(), 0);

    measuredVisualizer = new PivotVis("measured pivot scoral", Color.kAzure);
    setpointVisualizer = new PivotVis("setpoint pivot scoral", Color.kGreen);

    updateTunableNumbers();
  }

  public void setBrakeMode(boolean bool) {
    coralScorerArm.setBrakeMode(bool);
  }

  public double getArmPositionDegs() {
    return csaInputs.positionDegs;
  }

  public boolean atGoal(double threshold) {
    return (Math.abs(csaInputs.positionDegs - goalDegrees) <= threshold);
  }

  private double getArmError() {
    return csaInputs.positionSetpointDegs - csaInputs.positionDegs;
  }

  public void setPositionDegs(double positionDegs, double velocityDegsPerSec) {
    // positionDegs = MathUtil.clamp(positionDegs, 33, 120);
    coralScorerArm.setPositionSetpointDegs(
        positionDegs, armFFModel.calculate(positionDegs, velocityDegsPerSec));
  }

  public void armStop() {
    coralScorerArm.stop();
  }

  public void setArmGoal(double goalDegrees) {
    this.goalDegrees = goalDegrees;
    armGoalStateDegrees = new TrapezoidProfile.State(goalDegrees, 2);
  }

  public void setArmCurrent(double currentDegrees) {
    armCurrentStateDegrees = new TrapezoidProfile.State(currentDegrees, 0);
  }

  public Command setArmTarget(double goalDegrees, double thresholdDegrees) {
    // TODO: Change the wait time to an accurate value
    return new InstantCommand(() -> setArmGoal(goalDegrees), this)
        .until(() -> atGoal(thresholdDegrees))
        .withTimeout(5);
  }

  @Override
  public void periodic() {
    coralScorerArm.updateInputs(csaInputs);
    measuredVisualizer.update(armCurrentStateDegrees.position);
    armCurrentStateDegrees =
        armProfile.calculate(
            SubsystemConstants.LOOP_PERIOD_SECONDS, armCurrentStateDegrees, armGoalStateDegrees);

    setPositionDegs(armCurrentStateDegrees.position, armCurrentStateDegrees.velocity);

    Logger.processInputs("Coral Arm", csaInputs);
    Logger.recordOutput("arm error", getArmError());

    Logger.recordOutput("arm goal", goalDegrees);
    // This method will be called once per scheduler run
    measuredVisualizer.update(armCurrentStateDegrees.position);
    setpointVisualizer.update(armGoalStateDegrees.position);

    updateTunableNumbers();
  }

  private void updateTunableNumbers() {
    if (kP.hasChanged(hashCode())) {
      coralScorerArm.configurePID(kP.get(), 0, 0);
    }
    // if (kG.hasChanged(hashCode()) || kV.hasChanged(hashCode())) {
    //   armFFModel = new ArmFeedforward(0, kG.get(), kV.get(), 0);
    // }
  }
}
