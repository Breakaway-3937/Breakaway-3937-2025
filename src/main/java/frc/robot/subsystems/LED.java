// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LED extends SubsystemBase {
  private final CANdle candle;
  private final CANdleConfiguration config;
  private final GenericEntry ledStateEntry;
  private LEDStates ledState = LEDStates.DISABLED;
  private final Timer timer;
  private boolean flag, flag1;

  private final ColorFlowAnimation flow = new ColorFlowAnimation(0, 0, 0, 0, 0.1, Constants.NUM_LEDS, ColorFlowAnimation.Direction.Forward, 0);
  private final FireAnimation fire = new FireAnimation(1, 0.1, Constants.NUM_LEDS, 0.1, 0.1, false, 0);
  private final LarsonAnimation pocket = new LarsonAnimation(255, 0, 0, 0, 0.1, Constants.NUM_LEDS, LarsonAnimation.BounceMode.Center, 1);
  private final TwinkleOffAnimation twinkle = new TwinkleOffAnimation(0, 255, 0, 0, 0.1, Constants.NUM_LEDS, TwinkleOffPercent.Percent100, 0);
  private final SingleFadeAnimation fade = new SingleFadeAnimation(0, 0, 0, 0, 1, Constants.NUM_LEDS, 0);
  private final StrobeAnimation strobe = new StrobeAnimation(0, 0, 0, 0, 0.5, Constants.NUM_LEDS, 0);
  private final LarsonAnimation bad = new LarsonAnimation(179, 83, 97, 0, 0.1, Constants.NUM_LEDS, LarsonAnimation.BounceMode.Center, 1, 0);

  /** Creates a new LED.*/
  public LED() {
    candle = new CANdle(Constants.CANDLE_ID, Constants.CANIVORE_BUS);
    config = new CANdleConfiguration();

    config.statusLedOffWhenActive = false;
    config.disableWhenLOS = false;
    config.stripType = LEDStripType.RGBW;
    config.brightnessScalar = 0.25;
    config.vBatOutputMode = VBatOutputMode.On;
    config.v5Enabled = false;

    candle.configAllSettings(config);
    candle.setLEDs(0, 0, 0);

    timer = new Timer();
    timer.reset();
    timer.start();

    flag = false;
    flag1 = false;

    ledStateEntry = Shuffleboard.getTab("System").add("LEDState", getState().name()).withPosition(0, 1).getEntry();
  }

  public enum LEDStates {
    DISABLED,
    AUTONOMOUS,
    CLIMBED,
    FUNERAL,
    ALGAE_FULL,
    CORAL_FULL,
    BOT_EMPTY
  }

  public void setState(LEDStates ledState) {
    this.ledState  = ledState;
  }

  public LEDStates getState() {
    return ledState;
  }

  @Override
  public void periodic() {
    ledStateEntry.setString(getState().name());
    Logger.recordOutput("LED/LED State", getState().name());

    if(DriverStation.isFMSAttached()) {
      candle.configBrightnessScalar(1);
    }
    else {
      candle.configBrightnessScalar(0.25);
    }

    switch(ledState) {
      case DISABLED:
        if(timer.get() < 10) {
          candle.animate(fire);
        }
        else if(timer.get() < 20) {
          candle.animate(pocket);
        }
        else if(timer.get() < 30) {
          candle.animate(twinkle);
        }
        else {
          timer.reset();
          timer.start();
          twinkle.setR((int) (Math.random() * 255));
          twinkle.setG((int) (Math.random() * 255));
          twinkle.setB((int) (Math.random() * 255));
          pocket.setR((int) (Math.random() * 255));
          pocket.setG((int) (Math.random() * 255));
          pocket.setB((int) (Math.random() * 255));
        }
        break;
      case AUTONOMOUS:
        if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)) {
          flow.setR(0);
          flow.setB(254);
        }
        else {
          flow.setR(255);
          flow.setB(0);
        }
        candle.animate(flow);
        break;
      case CLIMBED:
        candle.clearAnimation(0);
        candle.setLEDs(0, 255, 0);
        break;
      case FUNERAL:
        candle.animate(bad);
        break;
      case ALGAE_FULL:
        if(!flag) {
          timer.reset();
          timer.start();
          flag = true;
          flag1 = false;
          strobe.setR(0);
          strobe.setG(0);
          strobe.setB(254);
          fade.setR(0);
          fade.setG(0);
          fade.setB(254);
        }
        else if(timer.get() > 3) {
          flag1 = true;
        }
        if(!flag1) {
          candle.animate(strobe);
        }
        else {
          candle.animate(fade);
        }
        break;
      case CORAL_FULL:
        if(!flag) {
          timer.reset();
          timer.start();
          flag = true;
          flag1 = false;
          strobe.setR(255);
          strobe.setG(255);
          strobe.setB(0);
          fade.setR(255);
          fade.setG(255);
          fade.setB(0);
        }
        else if(timer.get() > 3) {
          flag1 = true;
        }
        if(!flag1) {
          candle.animate(strobe);
        }
        else {
          candle.animate(fade);
        }
        break;
      case BOT_EMPTY:
        candle.clearAnimation(0);
        candle.setLEDs(255, 0, 0);
        break;
    }
  }
}
