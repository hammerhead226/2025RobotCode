// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.statemachines;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class AlignStateMachine {

  private final CANrange distanceSensor;
  private ALIGN_STATES targetState = ALIGN_STATES.NONE;

  public AlignStateMachine(CANrange distanceSensor) {
    this.distanceSensor = distanceSensor;
  }

  public void setTargetState(ALIGN_STATES state) {
    targetState = state;
    Logger.recordOutput("Align Target State", targetState);
  }

  public enum ALIGN_STATES {
    NONE,
    REEF,
    PROCESSOR,
    SOURCE,
  }

  public ALIGN_STATES getTargetState() {
    return targetState;
  }

  public Distance getThreshold() {
    switch (targetState) {
      case REEF:
        return Inches.of(8);
      case PROCESSOR:
        return Inches.of(7);
      case SOURCE:
        return Inches.of(6);
      case NONE:
        return Inches.of(0);
      default:
        return Inches.of(0);
    }
  }

  public boolean withinThreshold() {
    Distance distanceSensorReading = Meters.of(distanceSensor.getDistance().getValueAsDouble());
    distanceSensorReading.in(Inches);
    return distanceSensorReading.lt(getThreshold());
  }
}
