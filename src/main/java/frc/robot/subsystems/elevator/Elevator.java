package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SimConstants;
import frc.robot.constants.SubsystemConstants;
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

  private static final LoggedTunableNumber barkG = new LoggedTunableNumber("Bar/kG");

  // CHANGE THESE VALUES TO MATCH THE ELEVATOR
  private static final int maxVelocityExtender = 1;
  private static final int maxAccelerationExtender = 1;

  private TrapezoidProfile extenderProfile;
  private TrapezoidProfile.Constraints extenderConstraints =
      new TrapezoidProfile.Constraints(maxVelocityExtender, maxAccelerationExtender);
  private TrapezoidProfile.State extenderGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State extenderCurrent = new TrapezoidProfile.State();

  private double goal;
  private ElevatorFeedforward elevatorFFModel;

  public Elevator(ElevatorIO elevator) {
    this.elevator = elevator;

    switch (SimConstants.currentMode) {
      case REAL:
        kS.initDefault(0);
        kG.initDefault(0);
        kV.initDefault(0);
        kA.initDefault(0);

        kP.initDefault(0);
        kI.initDefault(0);

        barkG.initDefault(0);
        break;
      case REPLAY:
        kS.initDefault(0);
        kG.initDefault(0);
        kV.initDefault(0);
        kA.initDefault(0);

        kP.initDefault(0);
        kI.initDefault(0);

        barkG.initDefault(0);
        break;
      case SIM:
        kS.initDefault(0);
        kG.initDefault(0);
        kV.initDefault(0);
        kA.initDefault(0);

        kP.initDefault(1);
        kI.initDefault(0);

        barkG.initDefault(0);
        break;
      default:
        kS.initDefault(0);
        kG.initDefault(0);
        kV.initDefault(0);
        kA.initDefault(0);

        kP.initDefault(0);
        kI.initDefault(0);

        barkG.initDefault(0);
        break;
    }

    // CHANGE THIS VALUE TO MATCH THE ELEVATOR
    setExtenderGoal(1);
    extenderProfile = new TrapezoidProfile(extenderConstraints);
    extenderCurrent = extenderProfile.calculate(0, extenderCurrent, extenderGoal);

    updateTunableNumbers();
  }

  public boolean atGoal() {
    return (Math.abs(eInputs.elevatorPositionInch - goal)
        <= SubsystemConstants.ElevatorConstants.DEFAULT_THRESHOLD);
  }

  public double getElevatorPosition() {
    return eInputs.elevatorPositionInch;
  }

  private double getElevatorError() {
    return eInputs.positionSetpointInch - eInputs.elevatorPositionInch;
  }

  public boolean elevatorAtSetpoint(double thersholdInches) {
    return (Math.abs(getElevatorError()) <= thersholdInches);
  }

  public void setExtenderGoal(double setpoint) {
    goal = setpoint;
    extenderGoal = new TrapezoidProfile.State(setpoint, 0);
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
    return extenderGoal.position == SubsystemConstants.ElevatorConstants.EXTEND_SETPOINT_INCH;
  }

  public Command setElevatorTarget(double goalInches, double thersholdInches) {

    return new InstantCommand(() -> setExtenderGoal(goalInches), this)
        .until(() -> elevatorAtSetpoint(thersholdInches));
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Alliance", DriverStation.getAlliance().isPresent());

    elevator.updateInputs(eInputs);

    extenderCurrent =
        extenderProfile.calculate(
            SubsystemConstants.LOOP_PERIOD_SECONDS, extenderCurrent, extenderGoal);

    setPositionExtend(extenderCurrent.position, extenderCurrent.velocity);

    Logger.processInputs("Elevator", eInputs);

    updateTunableNumbers();
  }

  private void updateTunableNumbers() {
    if (kP.hasChanged(hashCode()) || kI.hasChanged(hashCode())) {
      elevator.configurePID(kP.get(), kI.get(), 0);
    }
    if (kS.hasChanged(hashCode())
        || kG.hasChanged(hashCode())
        || kV.hasChanged(hashCode())
        || kA.hasChanged(hashCode())) {
      elevatorFFModel = new ElevatorFeedforward(kS.get(), kG.get(), kV.get(), kA.get());
    }
  }
}
