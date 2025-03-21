package frc.robot.subsystems.scoral;

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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.SubsystemConstants;
import frc.robot.subsystems.commoniolayers.ArmIO;
import org.littletonrobotics.junction.Logger;

public class ScoralArmIOTalonFX implements ArmIO {
  private final TalonFX leader;
  private final CANcoder scoralCoder;
  private double positionSetpointDegs;

  private double startAngleDegs;

  private StatusSignal<Angle> leaderPositionRotations;
  private final StatusSignal<AngularVelocity> velocityDegsPerSec;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> statorCurrentAmps;
  private final StatusSignal<Current> supplyCurrentAmps;

  public ScoralArmIOTalonFX(int leadID, int canCoderID) {
    CANcoderConfiguration coderConfig = new CANcoderConfiguration();
    // change?
    coderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    // OFFSET IS IN ROTATIONS
    // coderConfig.MagnetSensor.withMagnetOffset(Units.degreesToRotations(58));

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = SubsystemConstants.ScoralArmConstants.CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable =
        SubsystemConstants.ScoralArmConstants.CURRENT_LIMIT_ENABLED;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.SensorToMechanismRatio = 1;
    leader = new TalonFX(leadID);
    scoralCoder = new CANcoder(canCoderID);

    leader.getConfigurator().apply(config);
    scoralCoder.getConfigurator().apply(coderConfig);

    if (scoralCoder.isConnected()) {
      leader.setPosition(
          (scoralCoder.getAbsolutePosition().getValueAsDouble() - Units.degreesToRotations(57 - 12))
              * SubsystemConstants.ScoralArmConstants.ARM_GEAR_RATIO);
    } else {
      leader.setPosition(
          Units.degreesToRotations(SubsystemConstants.ScoralArmConstants.STOW_SETPOINT_DEG)
              * SubsystemConstants.ScoralArmConstants.ARM_GEAR_RATIO);
    }

    leaderPositionRotations = leader.getPosition();
    velocityDegsPerSec = leader.getVelocity();
    appliedVolts = leader.getMotorVoltage();
    statorCurrentAmps = leader.getStatorCurrent();
    supplyCurrentAmps = leader.getSupplyCurrent();

    // leader.get

    positionSetpointDegs = SubsystemConstants.ScoralArmConstants.STOW_SETPOINT_DEG;

    Logger.recordOutput("start angle", startAngleDegs);

    leader.optimizeBusUtilization();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        leaderPositionRotations,
        velocityDegsPerSec,
        appliedVolts,
        statorCurrentAmps,
        supplyCurrentAmps);

    // setBrakeMode(false);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leaderPositionRotations,
        velocityDegsPerSec,
        appliedVolts,
        statorCurrentAmps,
        supplyCurrentAmps);
    // Logger.recordOutput("scoral arm motor rotations",
    // leaderPositionRotations.getValueAsDouble());
    // Logger.recordOutput(
    //     "scoral arm motor setpoint",
    //     Units.degreesToRotations(this.positionSetpointDegs)
    //         * SubsystemConstants.ScoralArmConstants.ARM_GEAR_RATIO);
    inputs.positionDegs =
        Units.rotationsToDegrees(leaderPositionRotations.getValueAsDouble())
            / SubsystemConstants.ScoralArmConstants.ARM_GEAR_RATIO;

    inputs.velocityDegsPerSec = Units.rotationsToDegrees(velocityDegsPerSec.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.positionSetpointDegs = positionSetpointDegs;

    // Logger.recordOutput(
    //     "Debug Scoral Arm/Motor Stator Current", leader.getStatorCurrent().getValueAsDouble());
    // Logger.recordOutput(
    //     "Debug Scoral Arm/Motor Supply Current", leader.getSupplyCurrent().getValueAsDouble());

    Logger.recordOutput(
        "cancoder arm position degs",
        Units.rotationsToDegrees(
            scoralCoder.getAbsolutePosition().getValueAsDouble()
                - Units.degreesToRotations(57 - 12)));
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

    // leader.setVoltage(1);
    leader.setControl(
        new PositionVoltage(
                Units.degreesToRotations(positionDegs)
                    * SubsystemConstants.ScoralArmConstants.ARM_GEAR_RATIO)
            .withFeedForward(ffVolts)); // CHECK FOR STOW ANGLE (positionDegs - 59)
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }

  @Override
  public void stop() {
    this.positionSetpointDegs = leaderPositionRotations.getValueAsDouble();
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
