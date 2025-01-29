package frc.robot.subsystems.commoniolayers;

import org.littletonrobotics.junction.AutoLog;

public interface DistanceIO {
    @AutoLog
    public static class DistanceSensorIOInputs {
        public double distance = 0;
        public int sustain = 0;
    }

    public default void updateInputs(DistanceSensorIOInputs inputs) {}

    public default void increaseSustain() {}

    public default void resetSustain() {}
    
}
