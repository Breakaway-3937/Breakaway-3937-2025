// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Soda;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrPepper extends SubsystemBase {
  private final TalonFX loader;
  private final TalonSRX thumb;
  private final CANrange sherlock, watson;
  private final GenericEntry coralFull, algaeFull, robotFull;

  /** Creates a new DrPepper.
   *  @since Ankle is no longer with us.
   */
  public DrPepper() {
    loader = new TalonFX(Constants.Soda.DrPepper.LOADER_CAN_ID);
    thumb = new TalonSRX(Constants.Soda.DrPepper.THUMB_CAN_ID);
    sherlock = new CANrange(Constants.Soda.DrPepper.SHERLOCK_CAN_ID);
    watson = new CANrange(Constants.Soda.DrPepper.WATSON_CAN_ID);

    configLoader();
    configThumb();
    configCANranges();

    coralFull = Shuffleboard.getTab("Soda").add("Coral Full", botFullCoral().getAsBoolean()).withPosition(3, 0).getEntry();
    algaeFull = Shuffleboard.getTab("Soda").add("Algae Full", botFullAlgae().getAsBoolean()).withPosition(4, 0).getEntry();
    robotFull = Shuffleboard.getTab("Driver").add("Robot Full", botFullAlgae().getAsBoolean() || botFullCoral().getAsBoolean()).withPosition(0, 0).withSize(10, 4).getEntry();
  }

  public Command runLoader() {
    return runOnce(() -> loader.set(1));
  }

  public Command runLoaderSlowly() {
    return runOnce(() -> loader.set(0.3));
  }

  public Command runLoaderReverse() {
    return runOnce(() -> loader.set(-1));
  }

  public Command runLoaderReverseTrough() {
    return runOnce(() -> loader.set(-0.3));
  }

  public Command runLoaderReverseSlowly() {
    return runOnce(() -> loader.set(-0.3));
  }

  public Command stopLoader() {
    return runOnce(() -> loader.stopMotor());
  }

  public Command runThumbForward() {
    return runOnce(() -> thumb.set(ControlMode.PercentOutput, 0.8));
  }

  public Command runThumbForwardSlowly() {
    return runOnce(() -> thumb.set(ControlMode.PercentOutput, 0.3));
  }

  public Command runThumbBackwardSlowly() {
    return runOnce(() -> thumb.set(ControlMode.PercentOutput, -0.3));
  }

  public Command stopThumb() {
    return runOnce(() -> thumb.set(ControlMode.PercentOutput, 0));
  }

  public Command runUntilFullCoral() {
    return runLoader().andThen(Commands.waitSeconds(0.5)).andThen(Commands.waitUntil(intakeFull())).andThen(stopLoader())
                      .andThen(runThumbBackwardSlowly()).andThen(Commands.waitUntil(() -> !botFullCoral().getAsBoolean()))
                      .andThen(stopThumb());
  }

  public Command runUntilFullAlgae() {
    return Commands.either(runLoaderReverseSlowly(), stopLoader(), botFullAlgae());
  }

  public BooleanSupplier intakeFull() {
    return () -> loader.getStatorCurrent().getValueAsDouble() > 40;
  }

  public BooleanSupplier botFullCoral() {
    return () -> sherlock.getIsDetected().getValue() && sherlock.getDistance().getValueAsDouble() > 0.06;
  }

  public BooleanSupplier botFullAlgae() {
    return () -> watson.getIsDetected().getValue() && watson.getDistance().getValueAsDouble() > 0.06;
  }

  public TalonFX getLoaderMotor() {
    return loader;
  }

  public void configThumb() {
    thumb.configFactoryDefault();

    TalonSRXConfiguration config = new TalonSRXConfiguration();

    config.peakCurrentDuration = 100;
    config.peakCurrentLimit = 50;
    config.continuousCurrentLimit = 35;

    thumb.configAllSettings(config);
    thumb.enableCurrentLimit(true);
  }

  public void configLoader() {
    loader.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Audio.AllowMusicDurDisable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.CurrentLimits.SupplyCurrentLimit = 80;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    config.CurrentLimits.SupplyCurrentLowerTime = 1;

    loader.getConfigurator().apply(config);
  }

  public void configCANranges() {
    sherlock.getConfigurator().apply(new CANrangeConfiguration());
    watson.getConfigurator().apply(new CANrangeConfiguration());

    CANrangeConfiguration config = new CANrangeConfiguration();
    config.ProximityParams.ProximityThreshold = 0.12;

    sherlock.getConfigurator().apply(config);

    config.ProximityParams.ProximityThreshold = 0.25;

    watson.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
    coralFull.setBoolean(botFullCoral().getAsBoolean());
    Logger.recordOutput("Soda/Coral Full", botFullCoral().getAsBoolean());

    algaeFull.setBoolean(botFullAlgae().getAsBoolean());
    Logger.recordOutput("Soda/Algae Full", botFullAlgae().getAsBoolean());

    robotFull.setBoolean(botFullAlgae().getAsBoolean() || botFullCoral().getAsBoolean());

    Logger.recordOutput("Soda/Loader Current", loader.getStatorCurrent().getValueAsDouble());

    if(Constants.DEBUG) {
      SmartDashboard.putNumber("Loader %", loader.get());
    }
  }
}
