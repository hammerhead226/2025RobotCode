package frc.robot.subsystems.newalgaeintake;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIntakeFlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public double flywheelVelocityRPM = 0;
    public double flywheelRotations;
    public double currentAmps = 0;
    public double appliedVolts = 0;
    public double velocitySetpointRPM = 0;
  }

  public default void updateInputs(FlywheelIOInputs inputs) {}

  public default void setVelocityRPS(double velocityRPS, double ffVolts) {}

  public default void stop() {}

  public default void configurePID(double kP, double kI, double kD) {}
}
