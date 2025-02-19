// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.coralIntake.flywheels;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.SimConstants;
import frc.robot.constants.SubsystemConstants;
import frc.robot.constants.SubsystemConstants.CoralState;
import frc.robot.subsystems.commoniolayers.FlywheelIO;
import frc.robot.subsystems.commoniolayers.FlywheelIOInputsAutoLogged;
import frc.robot.subsystems.newalgaeintake.FeederIOInputsAutoLogged;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class CoralIntakeFlywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final CoralIntakeSensorIO sensor;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private final CoralIntakeSensorIOInputsAutoLogged sInputs =
      new CoralIntakeSensorIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;
  private final SysIdRoutine sysId;

  private CoralState lastCoralState;

  private final FeederIOInputsAutoLogged feedInputs = new FeederIOInputsAutoLogged();

  /** Creates a new Flywheel. */
  public CoralIntakeFlywheel(FlywheelIO io, CoralIntakeSensorIO sensor, CoralState lastCoralState) {
    this.io = io;
    this.sensor = sensor;
    this.lastCoralState = lastCoralState;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (SimConstants.currentMode) {
      case REAL:
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        io.configurePID(0.0, 0.0, 0.0);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        io.configurePID(0.0, 0.0, 0.0);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Flywheel/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    sensor.updateInputs(sInputs);
    Logger.processInputs("Flywheel", inputs);
    Logger.processInputs("Sensor", sInputs);
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

    // Log flywheel setpoint
    Logger.recordOutput("Flywheel/SetpointRPM", velocityRPM);
  }

  public Command runVoltsCommmand(double volts) {

    return new InstantCommand(() -> runVolts(volts), this);
  }

  public Command runVelocityCommand(double velocityRPM) {

    return new InstantCommand(() -> runVelocity(velocityRPM), this);
  }

  public Command flywheelStop() {
    return new InstantCommand(() -> stop(), this);
  }

  /** Stops the flywheel. */
  public void stop() {
    io.stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  /** Returns the current velocity in radians per second. */
  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }

  public CoralState seesCoral() {
    Logger.recordOutput("see note val", "default");
    if ((sInputs.distance < SubsystemConstants.CORAL_DIST)) {
      Logger.recordOutput("see note val", "sensor");
      lastCoralState = CoralState.SENSOR;
      return CoralState.SENSOR;

    } else if (feedInputs.currentAmps > 13) {
      Logger.recordOutput("see note val", "current");
      lastCoralState = CoralState.CURRENT;
      return CoralState.CURRENT;

    } else {
      Logger.recordOutput("see note val", "no note");
      lastCoralState = CoralState.NO_CORAL;
      return CoralState.NO_CORAL;
    }
  }

  public CoralState getLastCoralState() {
    return lastCoralState;
  }
}
