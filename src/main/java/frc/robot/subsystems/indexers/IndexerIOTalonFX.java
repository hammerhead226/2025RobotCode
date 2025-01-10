package frc.robot.subsystems.indexers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class IndexerIOTalonFX implements IndexerIO {
  private final TalonFX talon;

  private final StatusSignal<Angle> positionRads;
  private final StatusSignal<AngularVelocity> velocityRadsPerSec;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> currentAmps;

  public IndexerIOTalonFX(int motorID) {
    talon = new TalonFX(motorID);

    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.CurrentLimits.SupplyCurrentLimit = 30.0;
    configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    talon.getConfigurator().apply(configs);

    positionRads = talon.getPosition();
    velocityRadsPerSec = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    currentAmps = talon.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, positionRads, velocityRadsPerSec, appliedVolts, currentAmps);
    talon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    BaseStatusSignal.refreshAll(positionRads, velocityRadsPerSec, appliedVolts, currentAmps);

    inputs.indexerPositionInch = positionRads.getValueAsDouble();
    inputs.indexerVelocityInchesPerSecond = velocityRadsPerSec.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
  }

  @Override
  public void runCharacterization(double volts) {
    talon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setPositionSetpoint(double positionDegs, double ffVolts) {
    talon.setControl(new PositionVoltage(Units.degreesToRotations(positionDegs)));
  }

  @Override
  public void stop() {
    talon.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    var config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;

    talon.getConfigurator().apply(config);
  }
}
