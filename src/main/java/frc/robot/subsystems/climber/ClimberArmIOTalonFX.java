package frc.robot.subsystems.climber;

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
import frc.robot.util.Conversions;
import org.littletonrobotics.junction.Logger;

public class ClimberArmIOTalonFX implements ClimberArmIO {

  private final TalonFX leader;
  private final CANcoder climbCoder;

  private double CLIMBER_ARM_GEAR_RATIO = 20 * 34 / 18;

  private double positionSetpointDegs;

  private double startAngleDegs;

  private final StatusSignal<Angle> leaderPositionDegs;
  private final StatusSignal<AngularVelocity> velocityDegsPerSec;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> currentAmps;

  public ClimberArmIOTalonFX(int leadID, int canCoderID) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit =
        SubsystemConstants.CoralScorerConstants.ScoralArmConstants.CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable =
        SubsystemConstants.CoralScorerConstants.ScoralArmConstants.CURRENT_LIMIT_ENABLED;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // config.Feedback.FeedbackRemoteSensorID = canCoderID;
    // config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

    CANcoderConfiguration coderConfig = new CANcoderConfiguration();

    coderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    // OFFSETS IN ROTATIONS
    coderConfig.MagnetSensor.withMagnetOffset(0);

    config.Feedback.FeedbackRemoteSensorID = canCoderID;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
    config.Feedback.SensorToMechanismRatio = 1.0;
    config.Feedback.RotorToSensorRatio = CLIMBER_ARM_GEAR_RATIO;

    leader = new TalonFX(leadID, SubsystemConstants.CANIVORE_ID_STRING);
    climbCoder = new CANcoder(canCoderID, SubsystemConstants.CANIVORE_ID_STRING);
    leader.getConfigurator().apply(config);
    climbCoder.getConfigurator().apply(coderConfig);

    leader.setPosition(Conversions.degreesToFalcon(startAngleDegs, CLIMBER_ARM_GEAR_RATIO));

    leaderPositionDegs = leader.getPosition();
    velocityDegsPerSec = leader.getVelocity();
    appliedVolts = leader.getMotorVoltage();
    currentAmps = leader.getStatorCurrent();

    // leader.get

    positionSetpointDegs = SubsystemConstants.ClimberConstants.STOW_SETPOINT_DEG;

    Logger.recordOutput("start angle", startAngleDegs);

    leader.optimizeBusUtilization();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100, leaderPositionDegs, velocityDegsPerSec, appliedVolts, currentAmps);

    // setBrakeMode(false);
  }

  @Override
  public void updateInputs(ClimberArmIOInputs inputs) {
    BaseStatusSignal.refreshAll(leaderPositionDegs, velocityDegsPerSec, appliedVolts, currentAmps);

    inputs.positionDegs =
        Conversions.falconToDegrees((leaderPositionDegs.getValueAsDouble()), CLIMBER_ARM_GEAR_RATIO)
            + SubsystemConstants.CoralScorerConstants.ScoralArmConstants.ARM_ZERO_ANGLE;

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
        new PositionVoltage(Conversions.degreesToFalcon(positionDegs, CLIMBER_ARM_GEAR_RATIO))
            .withFeedForward(ffVolts)); // CHECK FOR STOW ANGLE (positionDegs - 59)
  }

  @Override
  public void zeroPosition() {
    leader.setPosition(0);
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
