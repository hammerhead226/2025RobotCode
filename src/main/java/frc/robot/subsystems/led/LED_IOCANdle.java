// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.constants.SubsystemConstants;
import frc.robot.constants.SubsystemConstants.LED_STATE;

public class LED_IOCANdle implements LED_IO {
  LED_STATE ledState;

  CANdle candle;
  StrobeAnimation flashGreen =
      new StrobeAnimation(0, 204, 0, 0, 0.01, SubsystemConstants.LEDConstants.NUMBER_LEDS);
  StrobeAnimation flashRed =
      new StrobeAnimation(204, 0, 0, 0, 0.01, SubsystemConstants.LEDConstants.NUMBER_LEDS);
  StrobeAnimation flashBlue =
      new StrobeAnimation(0, 0, 255, 0, 0.01, SubsystemConstants.LEDConstants.NUMBER_LEDS);
  StrobeAnimation flashYellow =
      new StrobeAnimation(255, 255, 0, 0, 0.01, SubsystemConstants.LEDConstants.NUMBER_LEDS);
  StrobeAnimation flashWhite =
      new StrobeAnimation(255, 255, 255, 255, 0.01, SubsystemConstants.LEDConstants.NUMBER_LEDS);
  StrobeAnimation halfFlashWhite = new StrobeAnimation(255, 255, 255, 255, 0.01, 28 + 24, 0);
  StrobeAnimation flashPurple = new StrobeAnimation(119, 0, 255, 0, 0.01, 57 + 24);
  StrobeAnimation flashOrange = new StrobeAnimation(255, 165, 0, 0, 0.01, 57 + 24);
  StrobeAnimation flashGrey = new StrobeAnimation(96, 96, 96, 0, 0.01, 57 + 24);
  StrobeAnimation flashPink = new StrobeAnimation(255, 204, 255, 0, 0.01, 57 + 24);
  StrobeAnimation flashBlack = new StrobeAnimation(0, 0, 0, 0, 0.01, 57 + 24);
  StrobeAnimation flashCyan = new StrobeAnimation(204, 255, 255, 0, 0.01, 57 + 24);

  FireAnimation rainbow =
      new FireAnimation(0.3, 0.03, SubsystemConstants.LEDConstants.NUMBER_LEDS, 0.1, 0.1);
  // ColorFlowAnimation rainbow = new ColorFlowAnimation(0, 0, 255, 0, 0.343, 57,
  // Direction.Forward);

  ColorFlowAnimation off = new ColorFlowAnimation(0, 0, 0, 0, 0.01, 0, Direction.Forward, 28);
  // ColorFlowAnimation wayBlue =
  //     new ColorFlowAnimation(0, 0, 240, 0, 0.01, SubsystemConstants.LEDConstants.NUMBER_LEDS,
  // Direction.Forward, 32);
  // ColorFlowAnimation wayYellow =
  //     new ColorFlowAnimation(255, 255, 0, 0, 0, SubsystemConstants.LEDConstants.NUMBER_LEDS,
  // Direction.Forward, 0);
  // ColorFlowAnimation wayRed = new ColorFlowAnimation(240, 0, 0, 0, 0.01,
  // SubsystemConstants.LEDConstants.NUMBER_LEDS, Direction.Forward, 28);
  // ColorFlowAnimation wayGreen =
  //     new ColorFlowAnimation(0, 240, 0, 0, 0.01, SubsystemConstants.LEDConstants.NUMBER_LEDS / 2,
  // Direction.Forward, 28);

  public LED_IOCANdle(int channel, String CANBUS) {
    // led = new Spark(channel);
    candle = new CANdle(channel, CANBUS);
    ledState = SubsystemConstants.LED_STATE.BLUE;

    CANdleConfiguration configs = new CANdleConfiguration();
    // CANdleControlFrame.CANdle_Control_1_General(0x4000);
    configs.stripType = LEDStripType.RGB;
    configs.brightnessScalar = 0.8;

    candle.configAllSettings(configs);
    // setColor(LED_STATE.OFF);

    setLEDState(ledState);
  }

  @Override
  public void updateInputs(LED_IOInputs inputs) {
    inputs.ledState = ledState;
    // inputs.currentAmps = candle.getCurrent();
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
    switch (ledState) {
      case RED:
        // TODO:: reef level selected
        candle.clearAnimation(0);
        candle.setLEDs(255, 0, 0, 0, 0, SubsystemConstants.LEDConstants.NUMBER_LEDS);
        break;
      case BLUE:
        // TODO:: DEFAULT COLOR and when hanging
        candle.clearAnimation(0);
        candle.setLEDs(0, 0, 255, 0, 0, SubsystemConstants.LEDConstants.NUMBER_LEDS);
        break;
      case YELLOW:
        // TODO:: PROCESSOR
        candle.clearAnimation(0);
        candle.setLEDs(255, 255, 0, 0, 0, SubsystemConstants.LEDConstants.NUMBER_LEDS);
        break;
      case GREY:
        // TODO:: SOURCE
        candle.clearAnimation(0);
        candle.setLEDs(137, 129, 123);
        break;
      case GREEN:
        candle.clearAnimation(0);
        candle.setLEDs(0, 255, 0, 0, 0, SubsystemConstants.LEDConstants.NUMBER_LEDS);
        break;
      case PURPLE:
        // TODO:: INTAKE ALGAE FROM REEF
        candle.clearAnimation(0);
        candle.setLEDs(255, 0, 255);
        break;
      case PINK_LAVENDER:
        candle.clearAnimation(0);
        candle.setLEDs(209, 177, 200);
        break;
      case PAPAYA_ORANGE:
        candle.clearAnimation(0);
        candle.setLEDs(255, 30, 0);
        break;
      case FLASHING_GREEN:
        // TODO:: SCORING
        candle.animate(flashGreen, 0);
        break;
      case FLASHING_RED:
        // TODO:: auto align to reef
        candle.animate(flashRed, 0);
        break;
      case FLASHING_BLUE:
        candle.animate(flashBlue, 0);
        break;
      case FIRE:
        // TODO:: DISABLED | FIRE
        candle.animate(rainbow, 0);
        break;
      case OFF:
        candle.clearAnimation(0);
        candle.setLEDs(0, 0, 0, 0, 0, SubsystemConstants.LEDConstants.NUMBER_LEDS);
        break;
      default:
        break;
    }
  }

  @Override
  public void setLEDColors(Color8Bit[] colors) {
    for (int i = 0; i < Math.min(colors.length, SubsystemConstants.LEDConstants.NUMBER_LEDS); i++) {
      candle.setLEDs(colors[i].red, colors[i].green, colors[i].blue, 0, i, 1);
    }
  }
}
