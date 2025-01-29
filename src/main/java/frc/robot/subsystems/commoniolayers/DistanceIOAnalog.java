package frc.robot.subsystems.commoniolayers;

import edu.wpi.first.wpilibj.AnalogInput;

public class DistanceIOAnalog implements DistanceIO {
  private final AnalogInput proxSensor;
  private int sustain;

  public DistanceIOAnalog() {
    this.proxSensor = new AnalogInput(0);
    sustain = 0;
  }

  @Override
  public void updateInputs(DistanceSensorIOInputs inputs) {
    inputs.distance = proxSensor.getValue();
    inputs.sustain = this.sustain;
  }

  @Override
  public void increaseSustain() {
    sustain++;
  }

  @Override
  public void resetSustain() {
    sustain = 0;
  }
}
