// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.constants.SubsystemConstants;

/** Add your docs here. */
public class ElevatorIOSim implements ElevatorIO {

  // SIM VARIABLES (CHANGE)
  private int gearBoxMotorCount = 2;
  private int gearing = 12;
  private double carriageMassKg = 3.18;
  private double drumRadiusMeters = 1;
  private double minHeightMeters = 0;
  private double maxHeightMeters = 1.9;
  private boolean simulateGravity = true;
  private double initialPositionMeters = 0.0;

  private final DCMotor simGearbox = DCMotor.getKrakenX60Foc(gearBoxMotorCount);
  private ElevatorSim sim =
      new ElevatorSim(
          simGearbox,
          gearing,
          carriageMassKg,
          drumRadiusMeters,
          minHeightMeters,
          maxHeightMeters,
          simulateGravity,
          initialPositionMeters);
  private PIDController pid = new PIDController(0, 0, 0);

  private double positionInch = 0;
  private double velocityInchesPerSecond = 0;
  private double appliedVolts = 0;
  private double currentAmps = 0;
  private double positionSetpointInches = 0;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    positionSetpointInches = pid.getSetpoint();

    appliedVolts +=
        MathUtil.clamp(
            pid.calculate(Units.metersToInches(sim.getPositionMeters()), positionSetpointInches),
            -12.0,
            12);

    sim.setInputVoltage(appliedVolts);

    positionInch = Units.metersToInches(sim.getPositionMeters());
    velocityInchesPerSecond = Units.metersToInches(sim.getVelocityMetersPerSecond());
    currentAmps = sim.getCurrentDrawAmps();

    inputs.positionSetpointInch = positionSetpointInches;
    inputs.appliedVolts = appliedVolts;
    inputs.elevatorPositionInch = positionInch;
    inputs.elevatorVelocityInchesPerSecond = velocityInchesPerSecond;
    inputs.currentAmps = currentAmps;

    sim.update(SubsystemConstants.LOOP_PERIOD_SECONDS);
  }

  @Override
  public void runCharacterization(double volts) {
    sim.setInputVoltage(volts);
  }

  @Override
  public void setPositionSetpoint(double position, double ffVolts) {
    appliedVolts = ffVolts;
    pid.setSetpoint(position);
  }

  @Override
  public void stop() {
    appliedVolts = 0;
    pid.setSetpoint(Units.metersToInches(sim.getPositionMeters()));
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}
