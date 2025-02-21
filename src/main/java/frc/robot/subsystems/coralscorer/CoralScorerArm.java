package frc.robot.subsystems.coralscorer;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
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
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class CoralScorerArm extends SubsystemBase {
  private final ArmIO coralScorerArm;
  private final ArmIOInputsAutoLogged csaInputs = new ArmIOInputsAutoLogged();

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
  public CoralScorerArm(ArmIO arm) {
    this.coralScorerArm = arm;
    switch (SimConstants.currentMode) {
      case REAL:
        kG.initDefault(0);
        kV.initDefault(0.02);
        kP.initDefault(0);
        kA.initDefault(0);
        kS.initDefault(0);
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
        kG.initDefault(0.33);
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
    maxVelocityDegPerSec = 9; // was at 90
    maxAccelerationDegPerSecSquared = 18; // was at 190
    // maxAccelerationDegPerSecSquared = 180;

    armConstraints =
        new TrapezoidProfile.Constraints(maxVelocityDegPerSec, maxAccelerationDegPerSecSquared);
    armProfile = new TrapezoidProfile(armConstraints);

    // setArmGoal(-90);
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

  public boolean hasReachedGoal(double goalDegs) {
    return (Math.abs(armCurrentStateDegrees.position - goalDegs) <= 2);
  }

  private double getArmError() {
    return csaInputs.positionSetpointDegs - csaInputs.positionDegs;
  }

  public void setPositionDegs(double positionDegs, double velocityDegsPerSec) {
    // positionDegs = MathUtil.clamp(positionDegs, 33, 120);
    coralScorerArm.setPositionSetpointDegs(
        positionDegs, armFFModel.calculate(positionDegs, velocityDegsPerSec));
  }

  public void setVolts(double volts) {
    coralScorerArm.setVoltage(0.1);
  }

  public void armStop() {
    coralScorerArm.stop();
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

  @AutoLogOutput(key = "arm")
  public Pose3d getElevatorPose() {
    return new Pose3d(
        0, 0.3, 1, new Rotation3d(new Rotation2d(Math.toRadians(armCurrentStateDegrees.position))));
  }

  // // state machine stuff

  // public void setWantedState(ScoralArmState state) {
  //   wantedState = state;
  // }

  // public void Stow() {
  //   setArmGoal(0);
  // }

  // public void goToSource() {
  //   setArmGoal(40);
  // }

  // public void gotoFirstLevel() {
  //   setArmGoal(FieldConstants.ReefHeight.L1.pitch);
  // }

  // public void gotoSecondLevel() {
  //   setArmGoal(FieldConstants.ReefHeight.L2.pitch);
  // }

  // public void gotoThirdLevel() {
  //   setArmGoal(FieldConstants.ReefHeight.L3.pitch);
  // }

  // public void gotoFourthLevel() {
  //   setArmGoal(FieldConstants.ReefHeight.L4.pitch);
  // }

  // public void gotoProcessorLevel() {
  //   setArmGoal(90);
  // }

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
    // state machine stuff

    //   switch (currentState) {
    //     case ZERO:
    //       Stow();
    //       break;
    //     case SOURCE:
    //       goToSource();
    //       break;
    //     case L1:
    //       gotoFirstLevel();
    //       break;
    //     case L2:
    //       gotoSecondLevel();
    //       break;
    //     case L3:
    //       gotoThirdLevel();
    //       break;
    //     case L4:
    //       gotoFourthLevel();
    //       break;
    //     case PROCESSOR:
    //       gotoProcessorLevel();
    //     default:
    //       Stow();
    //   }
  }

  private void updateTunableNumbers() {
    if (kP.hasChanged(hashCode()) || kI.hasChanged(hashCode())) {
      coralScorerArm.configurePID(kP.get(), kI.get(), 0);
    }
    if (kG.hasChanged(hashCode())
        || kV.hasChanged(hashCode())
        || kA.hasChanged(hashCode())
        || kS.hasChanged(hashCode())) {
      armFFModel = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());
    }
  }
}
