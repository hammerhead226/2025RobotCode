package frc.robot.subsystems.coralIntake;

import org.littletonrobotics.junction.AutoLog;

public interface CoralIntakePivotIO {
  @AutoLog
  public static class CoralIntakePivotIOInputs {
    public double velocityDegsPerSec = 0;
    public double positionDegs = 0;
    public double currentAmps = 0;
    public double appliedVolts = 0;
    public double positionSetpointDegs;

    public boolean gyroConnected = false;
    public double pitch = 0;
  }

  public default void updateInputs(CoralIntakePivotIOInputs inputs) {}

  public default void setBrakeMode(boolean bool) {}

  public default void setPositionSetpointDegs(double positionDegs, double ffVolts) {}

  public default void setVoltage(double volts) {}

  public default void stop() {}

  public default void configurePID(double kP, double kI, double kD) {}
}