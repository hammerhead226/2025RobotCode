package frc.robot.subsystems.coralIntake.flywheels;

import org.littletonrobotics.junction.AutoLog;

public interface CoralIntakeSensorIO {
  @AutoLog
  public static class CoralIntakeSensorIOInputs {
    public double distance = 0; // assuming distance or proximity sensor
    public boolean connected = false;
  }

  public default void updateInputs(CoralIntakeSensorIOInputs inputs) {}
}
