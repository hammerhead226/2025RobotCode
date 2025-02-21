package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.SubsystemConstants;
import frc.robot.constants.SubsystemConstants.LED_STATE;

public class LED_IOSim implements LED_IO {
  LED_STATE ledState;

  public LED_IOSim() {
    ledState = SubsystemConstants.LED_STATE.BLUE;
    setLEDState(ledState);
  }

  @Override
  public void updateInputs(LED_IOInputs inputs) {
    inputs.ledState = ledState;
  }

  @Override
  public void noBumpersPressed() {
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      setLEDState(LED_STATE.BLUE);
      // led.set(Constants.LEDConstants.COLOR_BLUE);

    } else {
      setLEDState(LED_STATE.RED);
      // led.set(Constants.LEDConstants.COLOR_RED);
    }
  }

  @Override
  public void setLEDState(LED_STATE state) {
    ledState = state;
  }
}
