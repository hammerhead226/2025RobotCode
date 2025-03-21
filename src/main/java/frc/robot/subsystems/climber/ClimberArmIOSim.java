// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.SubsystemConstants;

/** Add your docs here. */
public class ClimberArmIOSim implements ClimberArmIO {

  // SIM VARIABLES (CHANGE)
  private int gearBoxMotorCount = 1;
  private double gearing = 20;
  private double armLength = Units.inchesToMeters(1); // CHANGE PER ARM
  private double momentOfInertia = SingleJointedArmSim.estimateMOI(armLength, 1); // CHANGE PER ARM
  private double minAngleRadians = 0;
  private double maxAngleRadians = Math.PI;
  private boolean simulateGravity = true;
  private double startingAngleRads = 0.0;

  private boolean closedLoop = true;

  private final DCMotor armGearbox = DCMotor.getFalcon500(gearBoxMotorCount);
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          armGearbox,
          gearing,
          momentOfInertia,
          armLength,
          minAngleRadians,
          maxAngleRadians,
          simulateGravity,
          startingAngleRads);
  private final PIDController pid = new PIDController(0, 0, 0);

  private double currentAmps = 0.0;
  private double appliedVolts = 0.0;
  private double velocityRadsPerSec = 0.0;
  private double positionRads = 0.0;
  private double positionSetpointRads = 0.0;

  private double clampedValueLowVolts = -12.0;
  private double clampedValueHighVolts = 12.0;

  @Override
  public void updateInputs(ClimberArmIOInputs inputs) {
    positionSetpointRads = pid.getSetpoint();
    if (closedLoop) {
      appliedVolts =
          MathUtil.clamp(
              pid.calculate(sim.getAngleRads(), positionSetpointRads),
              clampedValueLowVolts,
              clampedValueHighVolts);

      sim.setInputVoltage(appliedVolts);
    }
    positionRads = sim.getAngleRads();
    velocityRadsPerSec = sim.getVelocityRadPerSec();
    currentAmps = sim.getCurrentDrawAmps();

    inputs.positionSetpointDegs = Math.toDegrees(positionSetpointRads);
    inputs.appliedVolts = appliedVolts;
    inputs.positionDegs = Math.toDegrees(positionRads);
    inputs.velocityDegsPerSec = Math.toDegrees(velocityRadsPerSec);
    inputs.statorCurrentAmps = currentAmps;

    sim.update(SubsystemConstants.LOOP_PERIOD_SECONDS);
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = volts;
    sim.setInputVoltage(volts);
  }

  @Override
  public void setPositionSetpointDegs(double positionDegs, double ffVolts) {
    closedLoop = true;
    appliedVolts = ffVolts;
    pid.setSetpoint(Math.toRadians(positionDegs));
  }

  @Override
  public void stop() {
    appliedVolts = 0;
    pid.setSetpoint(sim.getAngleRads());
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}
