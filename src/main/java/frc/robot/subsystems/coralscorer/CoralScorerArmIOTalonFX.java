package frc.robot.subsystems.coralscorer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.SubsystemConstants;
import frc.robot.subsystems.commoniolayers.ArmIO;
import frc.robot.util.Conversions;
import org.littletonrobotics.junction.Logger;

public class CoralScorerArmIOTalonFX implements ArmIO {
  private final TalonFX leader;
  private final CANcoder scoralCoder;
  private double positionSetpointDegs;

  private double startAngleDegs;

  private final StatusSignal<Angle> leaderPositionDegs;
  private final StatusSignal<AngularVelocity> velocityDegsPerSec;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> currentAmps;

  public CoralScorerArmIOTalonFX(int leadID, int canCoderID) {
    CANcoderConfiguration coderConfig = new CANcoderConfiguration();
    // change?
    coderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    // OFFSET IS IN ROTATIONS
    coderConfig.MagnetSensor.withMagnetOffset(0);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit =
        SubsystemConstants.CoralScorerConstants.CoralScorerArmConstants.CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable =
        SubsystemConstants.CoralScorerConstants.CoralScorerArmConstants.CURRENT_LIMIT_ENABLED;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.Feedback.FeedbackRemoteSensorID = canCoderID;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.SensorToMechanismRatio = 1;
    config.Feedback.RotorToSensorRatio =
        SubsystemConstants.CoralScorerConstants.CoralScorerArmConstants.ARM_GEAR_RATIO;

    leader = new TalonFX(leadID);
    scoralCoder = new CANcoder(canCoderID);

    leader.getConfigurator().apply(config);
    scoralCoder.getConfigurator().apply(coderConfig);

    leader.setPosition(
        Conversions.degreesToFalcon(
            startAngleDegs,
            SubsystemConstants.CoralScorerConstants.CoralScorerArmConstants.ARM_GEAR_RATIO));

    leaderPositionDegs = leader.getPosition();
    velocityDegsPerSec = leader.getVelocity();
    appliedVolts = leader.getMotorVoltage();
    currentAmps = leader.getStatorCurrent();

    // leader.get

    positionSetpointDegs =
        SubsystemConstants.CoralScorerConstants.CoralScorerArmConstants.STOW_SETPOINT_DEG;

    Logger.recordOutput("start angle", startAngleDegs);

    leader.optimizeBusUtilization();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100, leaderPositionDegs, velocityDegsPerSec, appliedVolts, currentAmps);

    // setBrakeMode(false);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    BaseStatusSignal.refreshAll(leaderPositionDegs, velocityDegsPerSec, appliedVolts, currentAmps);

    inputs.positionDegs =
        Conversions.falconToDegrees(
                (leaderPositionDegs.getValueAsDouble()),
                SubsystemConstants.CoralScorerConstants.CoralScorerArmConstants.ARM_GEAR_RATIO)
            + SubsystemConstants.CoralScorerConstants.CoralScorerArmConstants.ARM_ZERO_ANGLE;

    inputs.velocityDegsPerSec = velocityDegsPerSec.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
    inputs.positionSetpointDegs = positionSetpointDegs;
  }

  @Override
  public void setBrakeMode(boolean bool) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    if (bool) {
      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    } else {
      config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    }

    leader.getConfigurator().apply(config);
  }

  @Override
  public void setPositionSetpointDegs(double positionDegs, double ffVolts) {
    this.positionSetpointDegs = positionDegs;
    leader.setControl(
        new PositionVoltage(
                Conversions.degreesToFalcon(
                    positionDegs,
                    SubsystemConstants.CoralScorerConstants.CoralScorerArmConstants.ARM_GEAR_RATIO))
            .withFeedForward(0.1)); // CHECK FOR STOW ANGLE (positionDegs - 59)
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }

  @Override
  public void stop() {
    this.positionSetpointDegs = leaderPositionDegs.getValueAsDouble();
    leader.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    Slot0Configs config = new Slot0Configs();

    config.kP = kP;
    config.kI = kI;
    config.kD = kD;

    leader.getConfigurator().apply(config);
  }
}
