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
import org.littletonrobotics.junction.Logger;

public class ElevatorIOTalonFX implements ElevatorIO {
  private final TalonFX leader;
  private final TalonFX follower;
  private final CANrange distanceSensor;

  private double positionSetpoint;
  private final StatusSignal<Angle> elevatorPosition;
  private final StatusSignal<AngularVelocity> elevatorVelocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> currentAmps;
  private final StatusSignal<Distance> canRangeDistance;

  public ElevatorIOTalonFX(int lead, int follow, int canRangeID) {
    TalonFXConfiguration leadConfig = new TalonFXConfiguration();
    leadConfig.CurrentLimits.StatorCurrentLimit =
        SubsystemConstants.ElevatorConstants.CURRENT_LIMIT;
    leadConfig.CurrentLimits.StatorCurrentLimitEnable =
        SubsystemConstants.ElevatorConstants.CURRENT_LIMIT_ENABLED;
    leadConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leadConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    leadConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    leader = new TalonFX(lead, SubsystemConstants.CANIVORE_ID_STRING);
    follower = new TalonFX(follow, SubsystemConstants.CANIVORE_ID_STRING);
    distanceSensor = new CANrange(canRangeID, SubsystemConstants.CANIVORE_ID_STRING);

    leader.getConfigurator().apply(leadConfig);

    positionSetpoint = SubsystemConstants.ElevatorConstants.STOW_SETPOINT_INCH;

    follower.setControl(new Follower(lead, true));

    TalonFXConfiguration followerConfig = new TalonFXConfiguration();
    followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    follower.getConfigurator().apply(followerConfig);

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

    Logger.recordOutput(
        "Elevator CANRage Distance Inch", distanceSensor.getDistance().getValueAsDouble());

    inputs.positionInch =
        Conversions.motorRotToInches(
            elevatorPosition.getValueAsDouble(),
            5.5,
            SubsystemConstants.ElevatorConstants.ELEVATOR_GEAR_RATIO);

    inputs.elevatorVelocityInchesPerSecond =
        Conversions.motorRotToInches(
            elevatorVelocity.getValueAsDouble() * 60.,
            5.5,
            SubsystemConstants.ElevatorConstants.ELEVATOR_GEAR_RATIO);
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();

    inputs.positionSetpointInch = positionSetpoint;
  }

  @Override
  public void runCharacterization(double volts) {
    leader.setVoltage(volts);
  }
  // weird how we give the setpoint in meters and it sets it to meters in sim?
  // we'll have to see what the exact bug is but for now work with meters

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }

  @Override
  public void setPositionSetpoint(double position, double ffVolts) {
    this.positionSetpoint = position;

    leader.setControl(
        new PositionVoltage(
                Conversions.inchesToMotorRot(
                    position, 5.5, SubsystemConstants.ElevatorConstants.ELEVATOR_GEAR_RATIO))
            .withFeedForward(ffVolts));
  }

  @Override
  public void zeroElevator() {
    leader.setPosition(0);
  }

  @Override
  public void stop() {
    this.positionSetpoint = elevatorPosition.getValueAsDouble();
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
    leader.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);
  }
}
