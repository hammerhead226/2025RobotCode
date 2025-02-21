package frc.robot.subsystems.arms;

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
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.SubsystemConstants;
import frc.robot.subsystems.commoniolayers.ArmIO;
import frc.robot.util.Conversions;
import org.littletonrobotics.junction.Logger;

public class ArmIOTalonFX implements ArmIO {
  private final TalonFX motor;

  private final Pigeon2 pigeon;

  private double positionSetpointDegs;

  private double startAngleDegs;

  private final StatusSignal<Angle> positionDegs;
  private final StatusSignal<AngularVelocity> velocityDegsPerSec;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> currentAmps;
  private final StatusSignal<Angle> pitch;

  public ArmIOTalonFX(int leadID, int followID, int gyroID) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = SubsystemConstants.ArmConstants.CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable =
        SubsystemConstants.ArmConstants.CURRENT_LIMIT_ENABLED;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    motor = new TalonFX(leadID, SubsystemConstants.CANBUS);
    pigeon = new Pigeon2(gyroID, SubsystemConstants.CANBUS); // use encoder here
    pigeon.reset();

    pitch = pigeon.getRoll(); // rename and get abs encoder (replace)
    // turnAbsolutePosition = cancoder.getAbsolutePosition();

    // startAngleDegs = turnAbsolutePosition.getValueAsDouble();

    motor.setPosition(
        Conversions.degreesToFalcon(
            startAngleDegs, SubsystemConstants.ArmConstants.ARM_GEAR_RATIO));

    positionDegs = motor.getPosition();
    velocityDegsPerSec = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    currentAmps = motor.getStatorCurrent();

    // leader.get

    positionSetpointDegs = SubsystemConstants.ArmConstants.STOW_SETPOINT_DEG;

    Logger.recordOutput("start angle", startAngleDegs);

    pigeon.optimizeBusUtilization();
    motor.optimizeBusUtilization();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100, positionDegs, velocityDegsPerSec, appliedVolts, currentAmps, pitch);

    // setBrakeMode(false);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        positionDegs, velocityDegsPerSec, appliedVolts, currentAmps, pitch);
    inputs.gyroConnected = BaseStatusSignal.refreshAll(pitch).equals(StatusCode.OK);
    inputs.pitch = pitch.getValueAsDouble() + SubsystemConstants.ArmConstants.ARM_ZERO_ANGLE;
    inputs.positionDegs =
        Conversions.falconToDegrees(
                (positionDegs.getValueAsDouble()),
                SubsystemConstants.ArmConstants.ARM_GEAR_RATIO)
            + SubsystemConstants.ArmConstants.ARM_ZERO_ANGLE;

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

    motor.getConfigurator().apply(config);
  }

  @Override
  public void setPositionSetpointDegs(double positionDegs, double ffVolts) {
    this.positionSetpointDegs = positionDegs;
    motor.setControl(
        new PositionVoltage(
            Conversions.degreesToFalcon(
                positionDegs,
                SubsystemConstants.ArmConstants
                    .ARM_GEAR_RATIO))); // CHECK FOR STOW ANGLE (positionDegs - 59)
  }

  @Override
  public void stop() {
    this.positionSetpointDegs = motor.getPosition().getValueAsDouble();
    motor.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    Slot0Configs config = new Slot0Configs();

    config.kP = kP;
    config.kI = kI;
    config.kD = kD;

    motor.getConfigurator().apply(config);
  }
}
