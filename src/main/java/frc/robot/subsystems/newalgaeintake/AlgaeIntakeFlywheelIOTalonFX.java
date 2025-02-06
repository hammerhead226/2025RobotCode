package frc.robot.subsystems.newalgaeintake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.SubsystemConstants;

public class AlgaeIntakeFlywheelIOTalonFX implements AlgaeIntakeFlywheelIO {

  private final TalonFX flywheel;

  private final StatusSignal<AngularVelocity> flywheelVelocityRPS;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> currentAmps;
  private final StatusSignal<Angle> flywheelRotations;

  private double velocitySetpointRPS = 0;

  public AlgaeIntakeFlywheelIOTalonFX(int id) {

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit =
        SubsystemConstants.CoralScorerConstants.AlgaeScorerFlywheelConstants.CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable =
        SubsystemConstants.CoralScorerConstants.AlgaeScorerFlywheelConstants.CURRENT_LIMIT_ENABLED;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    flywheel = new TalonFX(id, SubsystemConstants.CANBUS);

    flywheel.getConfigurator().apply(config);

    flywheelVelocityRPS = flywheel.getVelocity();
    appliedVolts = flywheel.getMotorVoltage();
    currentAmps = flywheel.getStatorCurrent();
    flywheelRotations = flywheel.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100, flywheelVelocityRPS, appliedVolts, currentAmps, flywheelRotations);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    BaseStatusSignal.refreshAll(flywheelVelocityRPS, appliedVolts, currentAmps, flywheelRotations);

    inputs.flywheelRotations = flywheelRotations.getValueAsDouble();
    inputs.velocitySetpointRPM = velocitySetpointRPS * 60.;
    inputs.flywheelVelocityRPM = flywheelVelocityRPS.getValueAsDouble() * 60.;
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
  }

  @Override
  public void setVelocityRPS(double velocityRPS, double ffVolts) {
    this.velocitySetpointRPS = velocityRPS;
    flywheel.setControl(new VelocityVoltage(velocityRPS));
  }

  @Override
  public void stop() {
    flywheel.stopMotor();
    velocitySetpointRPS = 0;
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    Slot0Configs configs = new Slot0Configs();

    configs.kP = kP;
    configs.kI = kI;
    configs.kD = kD;

    flywheel.getConfigurator().apply(configs);
  }
}
