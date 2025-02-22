package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.SubsystemConstants;
import frc.robot.util.Conversions;

public class ElevatorIOTalonFX implements ElevatorIO {
  private final TalonFX leader;
  private final TalonFX follower;
  private final CANrange distanceSensor;

  private double carriagePositionSetpoint;
  private double firstStagePositionSetpoint;
  private final StatusSignal<Angle> elevatorPosition;
  private final StatusSignal<AngularVelocity> elevatorVelocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> currentAmps;
  private final StatusSignal<Distance> canRangeDistance;

  public ElevatorIOTalonFX(int lead, int follow, int canRangeID) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = SubsystemConstants.ElevatorConstants.CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable =
        SubsystemConstants.ElevatorConstants.CURRENT_LIMIT_ENABLED;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    leader = new TalonFX(lead, SubsystemConstants.CANIVORE_ID_STRING);
    follower = new TalonFX(follow, SubsystemConstants.CANIVORE_ID_STRING);
    distanceSensor = new CANrange(canRangeID, SubsystemConstants.CANIVORE_ID_STRING);

    leader.getConfigurator().apply(config);

    carriagePositionSetpoint = SubsystemConstants.ElevatorConstants.STOW_SETPOINT_INCH;

    follower.setControl(new Follower(lead, true));

    elevatorPosition = leader.getPosition();
    elevatorVelocity = leader.getVelocity();
    appliedVolts = leader.getMotorVoltage();
    currentAmps = leader.getStatorCurrent();
    canRangeDistance = distanceSensor.getDistance();
    BaseStatusSignal.setUpdateFrequencyForAll(
        100, elevatorPosition, elevatorVelocity, appliedVolts, currentAmps);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(elevatorPosition, elevatorVelocity, appliedVolts, currentAmps);
    inputs.carriagePositionInch =
        Conversions.firstStageToCarriageInches(
            Conversions.motorRotToInches(
                elevatorPosition.getValueAsDouble(),
                5.5,
                SubsystemConstants.ElevatorConstants.ELEVATOR_GEAR_RATIO));
    inputs.firstStagePositionInch =
        Conversions.motorRotToInches(
            elevatorPosition.getValueAsDouble(),
            5.5,
            SubsystemConstants.ElevatorConstants.ELEVATOR_GEAR_RATIO);
    // if (canRangeDistance.getValueAsDouble() <= 12) {
    //   inputs.elevatorPositionInch =
    //       Units.metersToInches(canRangeDistance.getValueAsDouble()) * 2 + 4;
    //   inputs.CANrangeDistanceInches =
    //       Units.metersToInches(canRangeDistance.getValueAsDouble()) * 2 + 4;
    // } else {

    //   inputs.elevatorPositionInch =
    //       2
    //               * Conversions.motorRotToInches(
    //                   elevatorPosition.getValueAsDouble(),
    //                   5.5,
    //                   SubsystemConstants.ElevatorConstants.ELEVATOR_GEAR_RATIO)
    //           + 8;
    //   // - 0.051
    //   // + 0.017;
    //   inputs.CANrangeDistanceInches =
    //       Units.metersToInches(canRangeDistance.getValueAsDouble()) * 2 + 4;
    // }

    inputs.elevatorVelocityInchesPerSecond =
        Conversions.motorRotToInches(
            elevatorVelocity.getValueAsDouble() * 60.,
            5.5,
            SubsystemConstants.ElevatorConstants.ELEVATOR_GEAR_RATIO);
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
    inputs.carriagePositionSetpointInch = carriagePositionSetpoint;
    inputs.firstStagePositionSetpointInch = firstStagePositionSetpoint;
  }

  @Override
  public void runCharacterization(double volts) {
    leader.setVoltage(volts);
  }
  // weird how we give the setpoint in meters and it sets it to meters in sim?
  // we'll have to see what the exact bug is but for now work with meters
  @Override
  public void setFirstStagePositionSetpoint(double position, double ffVolts) {
    this.firstStagePositionSetpoint = position;
    this.carriagePositionSetpoint = Conversions.firstStageToCarriageInches(position);

    leader.setControl(
        new PositionVoltage(
                Conversions.inchesToMotorRot(
                    position, 5.5, SubsystemConstants.ElevatorConstants.ELEVATOR_GEAR_RATIO))
            .withFeedForward(ffVolts));
  }

  @Override
  public void stop() {
    this.carriagePositionSetpoint = elevatorPosition.getValueAsDouble();
    leader.stopMotor();
  }

  @Override
  public void configurePIDF(
      double kP, double kI, double kD, double kS, double kG, double kV, double kA) {
    Slot0Configs config = new Slot0Configs();

    config.kP = kP;
    config.kI = kI;
    config.kD = kD;

    // config.GravityType = GravityTypeValue.Elevator_Static;
    // config.kG = kG;
    // config.kV = kV;

    leader.getConfigurator().apply(config);
  }

  @Override
  public void setBrakeMode(boolean brake) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    if (brake) {
      config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    } else {
      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    }
  }
}
