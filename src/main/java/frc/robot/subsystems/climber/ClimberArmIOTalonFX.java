package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
// import com.ctre.phoenix6.signrals.InvertedValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.SubsystemConstants;
import frc.robot.util.Conversions;
import org.littletonrobotics.junction.Logger;

public class ClimberArmIOTalonFX implements ClimberArmIO {
  private final TalonFX leader;
  private final TalonFX follower;

  private final Pigeon2 pigeon;

  private double positionSetpointDegs;

  private double startAngleDegs;

  private final StatusSignal<Angle> leaderPositionDegs;
  private final StatusSignal<AngularVelocity> velocityDegsPerSec;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> currentAmps;
  private final StatusSignal<Angle> pitch;

  public ClimberArmIOTalonFX(int leadID, int followID, int gyroID) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = SubsystemConstants.ClimberConstants.CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable =
        SubsystemConstants.ClimberConstants.CURRENT_LIMIT_ENABLED;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    leader = new TalonFX(leadID, SubsystemConstants.CANIVORE_ID_STRING);
    follower = new TalonFX(followID, SubsystemConstants.CANIVORE_ID_STRING);
    pigeon = new Pigeon2(gyroID, SubsystemConstants.CANIVORE_ID_STRING);
    pigeon.reset();

    leader.getConfigurator().apply(config);

    follower.setControl(new Follower(leadID, true));

    pitch = pigeon.getRoll();

    startAngleDegs = pitch.getValueAsDouble();

    leader.setPosition(
        Conversions.degreesToFalcon(
            startAngleDegs, SubsystemConstants.ClimberConstants.ARM_GEAR_RATIO));

    follower.setPosition(
        Conversions.degreesToFalcon(
            startAngleDegs, SubsystemConstants.ClimberConstants.ARM_GEAR_RATIO));

    leaderPositionDegs = leader.getPosition();
    velocityDegsPerSec = leader.getVelocity();
    appliedVolts = leader.getMotorVoltage();
    currentAmps = leader.getStatorCurrent();

    // leader.get

    positionSetpointDegs = SubsystemConstants.ClimberConstants.STOW_SETPOINT_DEG;

    Logger.recordOutput("start angle", startAngleDegs);

    pigeon.optimizeBusUtilization();
    leader.optimizeBusUtilization();
    follower.optimizeBusUtilization();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100, leaderPositionDegs, velocityDegsPerSec, appliedVolts, currentAmps, pitch);

    // setBrakeMode(false);
  }

  @Override
  public void updateInputs(ClimberArmIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leaderPositionDegs, velocityDegsPerSec, appliedVolts, currentAmps, pitch);
    inputs.gyroConnected = BaseStatusSignal.refreshAll(pitch).equals(StatusCode.OK);
    inputs.pitch = pitch.getValueAsDouble() + SubsystemConstants.ClimberConstants.ARM_ZERO_ANGLE;
    inputs.positionDegs =
        Conversions.falconToDegrees(
                (leaderPositionDegs.getValueAsDouble()),
                SubsystemConstants.ClimberConstants.ARM_GEAR_RATIO)
            + SubsystemConstants.ClimberConstants.ARM_ZERO_ANGLE;

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
    follower.getConfigurator().apply(config);
  }

  @Override
  public void setPositionSetpointDegs(double positionDegs, double ffVolts) {
    this.positionSetpointDegs = positionDegs;
    leader.setControl(
        new PositionVoltage(
            Conversions.degreesToFalcon(
                positionDegs,
                SubsystemConstants.ClimberConstants
                    .ARM_GEAR_RATIO))); // CHECK FOR STOW ANGLE (positionDegs - 59)
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
