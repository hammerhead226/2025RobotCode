package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.SubsystemConstants;

public class ClimberFeederIOTalonFX implements ClimberFeederIO {

  private final TalonFX feeder;
  private final TalonFX follower;

  private final StatusSignal<AngularVelocity> feederVelocityRPS;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> currentAmps;
  private final StatusSignal<Angle> feederRotations;

  private double velocitySetpointRPS = 0;

  public ClimberFeederIOTalonFX(int id, int id2) {

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit =
        SubsystemConstants.CoralScorerConstants.AlgaeScorerFlywheelConstants.CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable =
        SubsystemConstants.CoralScorerConstants.AlgaeScorerFlywheelConstants.CURRENT_LIMIT_ENABLED;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    feeder = new TalonFX(id, SubsystemConstants.CANBUS);
    follower = new TalonFX(id2, SubsystemConstants.CANBUS);
    feeder.getConfigurator().apply(config);

    follower.setControl(new Follower(id, false));

    feederVelocityRPS = feeder.getVelocity();
    appliedVolts = feeder.getMotorVoltage();
    currentAmps = feeder.getStatorCurrent();
    feederRotations = feeder.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100, feederVelocityRPS, appliedVolts, currentAmps, feederRotations);
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    BaseStatusSignal.refreshAll(feederVelocityRPS, appliedVolts, currentAmps, feederRotations);

    inputs.feederRotations = feederRotations.getValueAsDouble();
    inputs.velocitySetpointRPM = velocitySetpointRPS * 60.;
    inputs.feederVelocityRPM = feederVelocityRPS.getValueAsDouble() * 60.;
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
  }

  @Override
  public void setVelocityRPS(double velocityRPS, double ffVolts) {
    this.velocitySetpointRPS = velocityRPS;
    feeder.setControl(new VelocityVoltage(velocityRPS));
  }

  @Override
  public void stop() {
    feeder.stopMotor();
    velocitySetpointRPS = 0;
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    Slot0Configs configs = new Slot0Configs();

    configs.kP = kP;
    configs.kI = kI;
    configs.kD = kD;

    feeder.getConfigurator().apply(configs);
  }
}
