package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
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
  private static final int maxAccelerationExtender = 150;

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

  public Elevator(ElevatorIO elevator) {
    this.elevator = elevator;

    switch (SimConstants.currentMode) {
      case REAL:
        // kS.initDefault(0.17);
        kS.initDefault(0.12);
        kG.initDefault(0.2);
        kV.initDefault(0.1706);
        kA.initDefault(0);

        kP.initDefault(0.8);
        kI.initDefault(0);
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

    extenderProfile = new TrapezoidProfile(extenderConstraints);
    extenderCurrent = extenderProfile.calculate(0, extenderCurrent, extenderGoal);
    measuredVisualizer = new ElevatorVis("measured", Color.kRed);
    setpointVisualizer = new ElevatorVis("setpoint", Color.kGreen);

    updateTunableNumbers();
  }

  public void setBrake(boolean brake) {
    elevator.setBrakeMode(brake);
  }

  public boolean atGoal(double threshold) {
    return (Math.abs(eInputs.positionInch - goal) <= threshold);
  }

  public boolean hasReachedGoal(double goalInches) {
    return (Math.abs(eInputs.positionInch - goalInches) <= 3);
  }

  public double getElevatorPosition() {
    return eInputs.positionInch;
  }

  private double getElevatorError() {
    return eInputs.positionSetpointInch - eInputs.positionInch;
  }

  public boolean elevatorAtSetpoint(double thresholdInches) {
    return (Math.abs(getElevatorError()) <= thresholdInches);
  }

  // public double getCanRangeDistanceInches() {
  //   return eInputs.CANrangeDistanceInches;
  // }

  public void setVoltage(double volts) {
    elevator.setVoltage(volts);
  }

  public void zeroElevator() {
    elevator.zeroElevator();
  }

  public void setElevatorCurrent(double currentInches) {
    extenderCurrent = new TrapezoidProfile.State(currentInches, 0);
  }

  public void setElevatorGoal(double goal) {
    this.goal = goal;
    extenderGoal = new TrapezoidProfile.State(goal, 0);
  }

  public void setPositionExtend(double position, double velocity) {
    elevator.setPositionSetpoint(position, elevatorFFModel.calculate(velocity));
  }

  public void elevatorStop() {
    elevator.stop();
  }

  public void setConstraints(
      double maxVelocityMetersPerSec, double maxAccelerationMetersPerSecSquared) {
    extenderConstraints =
        new TrapezoidProfile.Constraints(
            maxVelocityMetersPerSec, maxAccelerationMetersPerSecSquared);
    extenderProfile = new TrapezoidProfile(extenderConstraints);
  }

  // TODO fix the fact this information is pipped through so many different classes
  /**
   * determines if the robot should drive slow to avoid tipping due to high center of mass
   * 
   * used in SuperStructure's shouldSlowMode()
   * which is read by RobotContainer to create a trigger
   * and that trigger is used exclusively to mirror the slow mode boolean in Drive
   * finally Drive's isSlowMode() is used in AdjustToReef and DriveCommands
   * 
   * @return whether the robot should be in "slow mode"
   */
  public boolean shouldSlowMode() {
    return extenderGoal.position >= 10;
  }

  // public Command setElevatorTarget(double elevatorGoalInches, double thresholdInches) {
  //   // TODO: Change the wait time to an accurate value
  //   return new InstantCommand(() -> setElevatorGoal(elevatorGoalInches), this)
  //       .andThen(new WaitUntilCommand(() -> atGoal(thresholdInches)))
  //       .withTimeout(2.5);
  // }

  public void brakeMode(boolean brake) {
    elevator.setBrakeMode(brake);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Alliance", DriverStation.getAlliance().isPresent());
    elevator.updateInputs(eInputs);
    updateTunableNumbers();

    measured.update(extenderCurrent.position);
    ScoralArm.measuredVisualizer.updateVertical(extenderCurrent.position);

    extenderCurrent =
        extenderProfile.calculate(
            SubsystemConstants.LOOP_PERIOD_SECONDS, extenderCurrent, extenderGoal);

    setPositionExtend(extenderCurrent.position, extenderCurrent.velocity);

    Logger.processInputs("Elevator", eInputs);

    // Logger.recordOutput("Debug Elevator/at Goal", atGoal(2));

    measuredVisualizer.update(4 + extenderCurrent.position * 2);
    setpointVisualizer.update(4 + extenderGoal.position * 2);

    ScoralArm.measuredVisualizer.updateVertical(2 * extenderCurrent.position + 2);
    ScoralArm.setpointVisualizer.updateVertical(2 * extenderGoal.position + 2);

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
