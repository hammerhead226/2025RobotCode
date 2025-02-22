package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SimConstants;
import frc.robot.constants.SubsystemConstants;
import frc.robot.subsystems.scoral.ScoralArm;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private final ElevatorIO elevator;

  private final ElevatorIOInputsAutoLogged eInputs = new ElevatorIOInputsAutoLogged();

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP");
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI");

  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG");
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV");
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA");

  // CHANGE THESE VALUES TO MATCH THE ELEVATOR

  // cut velocity and acceleration in half
  private static final int maxVelocityExtender = 170;
  private static final int maxAccelerationExtender = 100;

  private TrapezoidProfile extenderProfile;
  private TrapezoidProfile.Constraints extenderConstraints =
      new TrapezoidProfile.Constraints(maxVelocityExtender, maxAccelerationExtender);
  private TrapezoidProfile.State extenderGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State extenderCurrent = new TrapezoidProfile.State();

  private double goal;
  private ElevatorFeedforward elevatorFFModel;
  private final ElevatorVis measured;

  private ElevatorVis measuredVisualizer;
  private ElevatorVis setpointVisualizer;

  // public enum ElevatorState {
  //   ZERO,
  //   STOW,
  //   L1,
  //   L2,
  //   L3,
  //   L4,
  //   SOURCE,
  //   PROCESSOR
  // }

  // private ElevatorState wantedState = ElevatorState.STOW;
  // private ElevatorState currentState = ElevatorState.STOW;

  public Elevator(ElevatorIO elevator) {
    this.elevator = elevator;
    SmartDashboard.putNumber("Elevator kS Gains", 0.17);
    SmartDashboard.putNumber("Elevator kG Gains", 0.2);
    SmartDashboard.putNumber("Elevator kV Gains", 0.1706);
    SmartDashboard.putNumber("Elevator kA Gains", 0);
    SmartDashboard.putNumber("Elevator kP Gains", 0.5);
    SmartDashboard.putNumber("Elevator kI Gains", 0);
    switch (SimConstants.currentMode) {
      case REAL:
        kS.initDefault(SmartDashboard.getNumber("Elevator kS Gains", 0.17));
        kG.initDefault(SmartDashboard.getNumber("Elevator kG Gains", 0.2));
        kV.initDefault(SmartDashboard.getNumber("Elevator kV Gains", 0.1706));
        kA.initDefault(SmartDashboard.getNumber("Elevator kA Gains", 0));
        kP.initDefault(SmartDashboard.getNumber("Elevator kP Gains", 0.5));
        kI.initDefault(SmartDashboard.getNumber("Elevator kI Gains", 0));
        break;
      case REPLAY:
        kS.initDefault(0);
        kG.initDefault(0);
        kV.initDefault(0);
        kA.initDefault(0);

        kP.initDefault(0);
        kI.initDefault(0);
        break;
      case SIM:
        kS.initDefault(0.0);
        kG.initDefault(0.1);
        // kV.initDefault(0.55);
        kV.initDefault(0.055);
        kA.initDefault(0);

        kP.initDefault(11);
        // kP.initDefault(0.5);
        kI.initDefault(0);
        break;
      default:
        kS.initDefault(0);
        kG.initDefault(0);
        kV.initDefault(0);
        kA.initDefault(0);

        kP.initDefault(0);
        kI.initDefault(0);
        break;
    }
    measured = new ElevatorVis("measured", Color.kRed);

    // CHANGE THIS VALUE TO MATCH THE ELEVATOR
    // setExtenderGoal(1.3);
    extenderProfile = new TrapezoidProfile(extenderConstraints);
    extenderCurrent = extenderProfile.calculate(0, extenderCurrent, extenderGoal);

    // extenderProfile2 = new TrapezoidProfile(extenderConstraints2);

    measuredVisualizer = new ElevatorVis("measured", Color.kRed);
    setpointVisualizer = new ElevatorVis("setpoint", Color.kGreen);

    updateTunableNumbers();
  }

  public boolean atGoal() {
    return (Math.abs(extenderCurrent.position - goal)
        <= SubsystemConstants.ElevatorConstants.DEFAULT_THRESHOLD);
  }

  public boolean hasReachedGoal(double goalInches) {
    return (Math.abs(eInputs.elevatorPositionInch - goalInches)
        <= SubsystemConstants.ElevatorConstants.DEFAULT_THRESHOLD);
  }

  public double getElevatorPosition() {
    return eInputs.elevatorPositionInch;
  }

  private double getElevatorError() {
    return eInputs.positionSetpointInch - eInputs.elevatorPositionInch;
  }

  public boolean elevatorAtSetpoint(double thresholdInches) {
    return (Math.abs(getElevatorError()) <= thresholdInches);
  }

  public double getCanRangeDistanceInches() {
    return eInputs.CANrangeDistanceInches;
  }

  public Command zeroCommand(double volts) {
    return new InstantCommand(() -> elevator.runCharacterization(volts), this)
        .until(() -> getCanRangeDistanceInches() == 0 + 0.5);
  }

  public void setExtenderGoal(double goal) {
    this.goal = goal;
    extenderGoal = new TrapezoidProfile.State(goal, 0);
    // extenderGoal2 = new TrapezoidProfile.State(setpoint, 0);
  }

  public void setPositionExtend(double position, double velocity) {
    elevator.setPositionSetpoint(position, elevatorFFModel.calculate(velocity));
  }

  public void elevatorStop() {
    elevator.stop();
  }

  public double calculateAngle() {
    double angle = 0.0;
    return angle;
  }

  public void setConstraints(
      double maxVelocityMetersPerSec, double maxAccelerationMetersPerSecSquared) {
    extenderConstraints =
        new TrapezoidProfile.Constraints(
            maxVelocityMetersPerSec, maxAccelerationMetersPerSecSquared);
    extenderProfile = new TrapezoidProfile(extenderConstraints);
  }

  public boolean isExtended() {
    return extenderGoal.position >= 0.4;
  }

  public Command setElevatorTarget(double goalInches, double thresholdInches) {
    // TODO: Change the wait time to an accurate value
    return new InstantCommand(() -> setExtenderGoal(goalInches), this)
        .until(() -> elevatorAtSetpoint(thresholdInches))
        .withTimeout(5);
  }

  // @AutoLogOutput(key = "elevator")
  // public Pose3d getElevatorPose() {
  //   if (getElevatorstage2Pose().getZ() < extenderCurrent.position) {
  //     return new Pose3d(0, 0, extenderCurrent2.position + 0.5, new Rotation3d());
  //   } else {
  //     return new Pose3d(0, 0, extenderCurrent.position + 0.9, new Rotation3d());
  //   }
  // }

  // state stuff
  // public void setWantedState(ElevatorState wantedState) {
  //   this.wantedState = wantedState;
  // }

  public void breakMode(boolean brake) {
    elevator.setBrakeMode(brake);
  }

  // public ElevatorState handleStateTransitions() {
  //   return switch (wantedState) {
  //     case ZERO -> ElevatorState.ZERO;
  //     case STOW -> ElevatorState.STOW;
  //     case SOURCE -> ElevatorState.SOURCE;
  //     case L1 -> ElevatorState.L1;
  //     case L2 -> ElevatorState.L2;
  //     case L3 -> ElevatorState.L3;
  //     case L4 -> ElevatorState.L4;
  //     default -> ElevatorState.ZERO;
  //   };
  // }

  // // elevator factory
  // public void Stow() {
  //   setExtenderGoal(0);
  // }

  // public void goToSource() {
  //   setExtenderGoal(0);
  // }

  // public void gotoFirstLevel() {
  //   setExtenderGoal(FieldConstants.ReefHeight.L1.height);
  // }

  // public void gotoSecondLevel() {
  //   setExtenderGoal(FieldConstants.ReefHeight.L2.height);
  // }

  // public void gotoThirdLevel() {
  //   setExtenderGoal(FieldConstants.ReefHeight.L3.height);
  // }

  // public void gotoFourthLevel() {
  //   setExtenderGoal(FieldConstants.ReefHeight.L4.height);
  // }

  // public void gotoProcessorLevel() {
  //   setExtenderGoal(0);
  // }

  @Override
  public void periodic() {
    Logger.recordOutput("Alliance", DriverStation.getAlliance().isPresent());
    elevator.updateInputs(eInputs);
    updateTunableNumbers();
    // state logic
    // ElevatorState desiredState = wantedState;
    // if (wantedState != currentState) {
    //   currentState = wantedState;
    // }

    // switch (currentState) {
    //   case ZERO:
    //     Stow();
    //     break;
    //   case SOURCE:
    //     goToSource();
    //     break;
    //   case L1:
    //     gotoFirstLevel();
    //     break;
    //   case L2:
    //     gotoSecondLevel();
    //     break;
    //   case L3:
    //     gotoThirdLevel();
    //     break;
    //   case L4:
    //     gotoFourthLevel();
    //     break;
    //   case PROCESSOR:
    //     gotoProcessorLevel();
    //   default:
    //     Stow();
    // }

    measured.update(extenderCurrent.position);
    ScoralArm.measuredVisualizer.updateVertical(extenderCurrent.position);

    extenderCurrent =
        extenderProfile.calculate(
            SubsystemConstants.LOOP_PERIOD_SECONDS, extenderCurrent, extenderGoal);

    setPositionExtend(extenderCurrent.position, extenderCurrent.velocity);

    Logger.processInputs("Elevator", eInputs);

    measuredVisualizer.update(0.55 + extenderCurrent.position);
    setpointVisualizer.update(0.55 + extenderGoal.position);

    ScoralArm.measuredVisualizer.updateVertical(extenderCurrent.position + 0.1);
    ScoralArm.setpointVisualizer.updateVertical(extenderGoal.position + 0.1);
    // Logger.recordOutput("setpoint for elevator",)

    updateTunableNumbers();
  }

  private void updateTunableNumbers() {
    if (kP.hasChanged(hashCode()) || kI.hasChanged(hashCode())) {
      elevator.configurePIDF(kP.get(), kI.get(), 0, kS.get(), kG.get(), kV.get(), kA.get());
    }
    if (kS.hasChanged(hashCode())
        || kG.hasChanged(hashCode())
        || kV.hasChanged(hashCode())
        || kA.hasChanged(hashCode())) {
      elevatorFFModel = new ElevatorFeedforward(kS.get(), kG.get(), kV.get(), kA.get());
    }
  }
}
