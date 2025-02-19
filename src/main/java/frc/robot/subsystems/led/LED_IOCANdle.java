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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.constants.SubsystemConstants;
import frc.robot.constants.SubsystemConstants.LED_STATE;
import java.util.Random;

public class LED_IOCANdle implements LED_IO {
  LED_STATE ledState;

  public CANdle candle;
  public Color elastColor = new Color();
  Random rand = new Random();
        int coinflip = rand.nextInt() % 2;
  StrobeAnimation flashGreen = new StrobeAnimation(0, 204, 0, 0, 0.01, 57 + 24);
  StrobeAnimation flashRed = new StrobeAnimation(204, 0, 0, 0, 0.01, 57 + 24);
  StrobeAnimation flashBlue = new StrobeAnimation(0, 0, 255, 0, 0.01, 57 + 24);
  StrobeAnimation flashYellow = new StrobeAnimation(255, 255, 0, 0, 0.01, 57 + 24);
  StrobeAnimation flashWhite = new StrobeAnimation(255, 255, 255, 255, 0.01, 57 + 24);
  StrobeAnimation halfFlashWhite = new StrobeAnimation(255, 255, 255, 255, 0.01, 28 + 24, 0);
  StrobeAnimation flashPurple = new StrobeAnimation(119, 0, 200, 0, 0.01, 57 + 24);
  // StrobeAnimation flashOrange = new StrobeAnimation (255, 165 ,0, 0.01,57+24);
  StrobeAnimation flashOrange = new StrobeAnimation(255, 165, 0, 0, 0.01, 57 + 24);

  FireAnimation rainbow = new FireAnimation(0.3, 0.03, 57 + 24, 0.1, 0.1);
  // ColorFlowAnimation rainbow = new ColorFlowAnimation(0, 0, 255, 0, 0.343, 57,
  // Direction.Forward);

  ColorFlowAnimation off = new ColorFlowAnimation(0, 0, 0, 0, 0.01, 0, Direction.Forward, 28);
  ColorFlowAnimation wayBlue =
      new ColorFlowAnimation(0, 0, 240, 0, 0.01, 24, Direction.Forward, 32);
  ColorFlowAnimation wayYellow =
      new ColorFlowAnimation(255, 255, 0, 0, 0, 56, Direction.Forward, 0);
  ColorFlowAnimation wayRed = new ColorFlowAnimation(240, 0, 0, 0, 0.01, 28, Direction.Forward, 28);
  ColorFlowAnimation wayGreen =
      new ColorFlowAnimation(0, 240, 0, 0, 0.01, 28, Direction.Forward, 28);

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
    // candle.setLEDs(0, 0, 0);
    switch (ledState) {
      case RED:
        // TODO:: MANUAL INTAKE | RED
        candle.clearAnimation(0);
        candle.setLEDs(255, 0, 0, 0, 0, 57 + 24);
        elastColor = new Color(255, 0, 0);
        SmartDashboard.putString("LED Color", elastColor.toHexString());
        break;
      case BLUE:
        // TODO:: DEFAULT COLOR | BLUE
        candle.clearAnimation(0);
        candle.setLEDs(0, 0, 255, 0, 0, 57 + 24);
        elastColor = new Color(0, 0, 255);
        SmartDashboard.putString("LED Color", elastColor.toHexString());
        break;
      case FLASHING_PURPLE:
        candle.clearAnimation(0);
        candle.setLEDs(119, 0, 200, 0, 0, 57 + 24);
        elastColor = new Color(119, 0, 200);
        SmartDashboard.putString("LED Color", elastColor.toHexString());
        if (coinflip == 0) {
          SmartDashboard.putString("LED Color", elastColor.toHexString());
        } else {
          SmartDashboard.putString("LED Color", "#FFFFFF");
        }
        break;
      case YELLOW:
        candle.clearAnimation(0);
        // led.set(Constants.LEDConstants.COLOR_YELLOW);
        // candle.setLEDs(255, 255, 0, 0, 32, 25);
        candle.setLEDs(255, 255, 0, 0, 0, 57 + 24);
        elastColor = new Color(255, 255, 0);
        SmartDashboard.putString("LED Color", elastColor.toHexString());
        break;
      case VIOLET:
        // led.set(Constants.LEDConstants.COLOR_VIOLET);
        elastColor = new Color(200, 0, 200);
        SmartDashboard.putString("LED Color", elastColor.toHexString());
        break;
      case GREY:
        candle.clearAnimation(0);
        candle.setLEDs(137, 129, 123);
        elastColor = new Color(137, 129, 123);
        SmartDashboard.putString("LED Color", elastColor.toHexString());
        break;
      case GREEN:
        candle.clearAnimation(0);
        candle.setLEDs(0, 255, 0, 0, 0, 57 + 24);
        elastColor = new Color(0, 255, 0);
        SmartDashboard.putString("LED Color", elastColor.toHexString());
        break;

      case PURPLE:
        candle.clearAnimation(0);
        candle.setLEDs(255, 0, 255);
        elastColor = new Color(255, 0, 255);
        SmartDashboard.putString("LED Color", elastColor.toHexString());
        break;
      case PAPAYA_ORANGE:
        candle.clearAnimation(0);
        candle.setLEDs(255, 30, 0);
        elastColor = new Color(255, 30, 0);
        SmartDashboard.putString("LED Color", elastColor.toHexString());
        break;
      case WILLIAMS_BLUE:
        candle.clearAnimation(0);
        candle.setLEDs(0, 160, 222);
        elastColor = new Color(0, 160, 222);
        SmartDashboard.putString("LED Color", elastColor.toHexString());
        break;
      case HALF_FLASH_RED_HALF_FLASH_WHITE:
        // TODO:: INTAKING FROM SOURCE | HALF_FLASH_RED_HALF_FLASH_WHITE
        candle.clearAnimation(0);
        candle.animate(halfFlashWhite);
        // candle.setLEDs(255, 0, 0, 0, 32, 28);
        candle.setLEDs(255, 0, 0, 0, 32, 25);
        elastColor = new Color(255, 0, 0);
        SmartDashboard.putString("LED Color", elastColor.toHexString());
        break;
      case FLASHING_ORANGE:
        candle.animate(flashOrange, 0);
        elastColor = new Color(255, 30, 0);
        SmartDashboard.putString("LED Color", elastColor.toHexString());
        break;
      case FLASHING_WHITE:
        // TODO:: SIGNAL TO HUMAN PLAYER | FLASHING WHITE
        candle.animate(flashWhite, 0);
        elastColor = new Color(0, 0, 0);
        SmartDashboard.putString("LED Color", elastColor.toHexString());
        if (coinflip == 0) {
          SmartDashboard.putString("LED Color", elastColor.toHexString());
        } else {
          SmartDashboard.putString("LED Color", "#FFFFFF");
        }
        break;
      case FLASHING_YELLOW:
        candle.animate(flashYellow, 0);
        elastColor = new Color(255, 255, 0);
        SmartDashboard.putString("LED Color", elastColor.toHexString());
        if (coinflip == 0) {
          SmartDashboard.putString("LED Color", elastColor.toHexString());
        } else {
          SmartDashboard.putString("LED Color", "#FFFFFF");
        }
        break;
      case FLASHING_GREEN:
        // TODO:: AIMBOT | FLASHING GREEN
        candle.animate(flashGreen, 0);
        elastColor = new Color(0, 255, 0);
        SmartDashboard.putString("LED Color", elastColor.toHexString());
        if (coinflip == 0) {
          SmartDashboard.putString("LED Color", elastColor.toHexString());
        } else {
          SmartDashboard.putString("LED Color", "#FFFFFF");
        }
        break;
      case FLASHING_RED:
        // TODO:: TRAJECTORY INTAKE | FLASHING RED
        candle.animate(flashRed, 0);
        elastColor = new Color(255, 0, 0);
        SmartDashboard.putString("LED Color", elastColor.toHexString());
        if (coinflip == 0) {
          SmartDashboard.putString("LED Color", elastColor.toHexString());
        } else {
          SmartDashboard.putString("LED Color", "#FFFFFF");
        }
        break;
      case FLASHING_BLUE:
        candle.animate(flashBlue, 0);
        elastColor = new Color(0, 0, 255);
        SmartDashboard.putString("LED Color", elastColor.toHexString());
        if (coinflip == 0) {
          SmartDashboard.putString("LED Color", elastColor.toHexString());
        } else {
          SmartDashboard.putString("LED Color", "#FFFFFF");
        }
        break;
      case FIRE:
        // TODO:: DISABLED | FIRE
        candle.animate(rainbow, 0);
        elastColor = new Color(255, 30, 0);
        SmartDashboard.putString("LED Color", elastColor.toHexString());
        break;
      case OFF:
        candle.clearAnimation(0);
        candle.setLEDs(0, 0, 0, 0, 0, 57 + 24);
        elastColor = new Color(0, 0, 0);
        SmartDashboard.putString("LED Color", elastColor.toHexString());
        break;
      default:
        break;
    }
  }
}
