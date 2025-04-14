// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Soda;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrPepper extends SubsystemBase {
  private final TalonFX loader, thumb;
  private final CANrange sherlock, watson;
  private final GenericEntry coralFull, algaeFull, robotFull;
  private final BooleanSupplier isAlgae;
  private boolean algaeFlag; //Yay flag

  /** Creates a new DrPepper.
   *  @since Ankle is no longer with us.
   */
  public DrPepper(BooleanSupplier isAlgae) {
    loader = new TalonFX(Constants.Soda.DrPepper.LOADER_CAN_ID);
    thumb = new TalonFX(Constants.Soda.DrPepper.THUMB_CAN_ID);
    sherlock = new CANrange(Constants.Soda.DrPepper.SHERLOCK_CAN_ID);
    watson = new CANrange(Constants.Soda.DrPepper.WATSON_CAN_ID);

    this.isAlgae = isAlgae;
    
    configLoader();
    configThumb();
    configCANranges();

    coralFull = Shuffleboard.getTab("Soda").add("Coral Full", botFullCoral().getAsBoolean()).withPosition(3, 0).getEntry();
    algaeFull = Shuffleboard.getTab("Soda").add("Algae Full", botFullAlgae().getAsBoolean()).withPosition(4, 0).getEntry();
    robotFull = Shuffleboard.getTab("Driver").add("Robot Full", botFullAlgae().getAsBoolean() || botFullCoral().getAsBoolean()).withPosition(0, 0).withSize(10, 4).getEntry();

    algaeFlag = false;
  }

  public Command runLoader() {
    return runOnce(() -> loader.set(1));
  }

  public Command runLoaderSlowly() {
    return runOnce(() -> loader.set(0.3));
  }

  public Command runLoaderReverse() {
    return runOnce(() -> loader.set(-1)).andThen(runOnce(() -> algaeFlag = false));
  }

  public Command runLoaderReverseTrough() {
    return runOnce(() -> loader.set(-0.3)).andThen(runOnce(() -> algaeFlag = false));
  }

  public Command runLoaderReverseBarge() {
    return runOnce(() -> loader.set(-0.5)).andThen(runOnce(() -> algaeFlag = false));
  }

  public Command stopLoader() {
    return runOnce(() -> loader.stopMotor());
  }

  public Command runThumbForward() {
    return runOnce(() -> thumb.set(0.8));
  }

  public Command runThumbForwardL2L3() {
    return runOnce(() -> thumb.set(0.5));
  }

  public Command runThumbBackward() {
    return runOnce(() -> thumb.set(-0.8));
  }

  public Command runThumbForwardSlowly() {
    return runOnce(() -> thumb.set(0.15));
  }

  public Command runThumbBackwardSlowly() {
    return runOnce(() -> thumb.set(-0.15));
  }

  public Command stopThumb() {
    return runOnce(() -> thumb.set(0));
  }

  public Command center() {
    return Commands.either(Commands.either(stopThumb(), runThumbBackwardSlowly(), () -> watson.getIsDetected().getValue()), Commands.either(runThumbForwardSlowly(), stopThumb(), () -> watson.getIsDetected().getValue()), () -> sherlock.getIsDetected().getValue());
  }

  public Command autoCenter() {
    return Commands.either(Commands.either(stopThumb(), runThumbBackwardSlowly(), () -> watson.getIsDetected().getValue()), Commands.either(runThumbForwardSlowly(), Commands.none(), () -> watson.getIsDetected().getValue()), () -> sherlock.getIsDetected().getValue()).repeatedly().until(() -> sherlock.getIsDetected().getValue() && watson.getIsDetected().getValue());
  }

  public BooleanSupplier botFullCoral() {
    return () -> !isAlgae.getAsBoolean() && !botFullAlgae().getAsBoolean() && (sherlock.getIsDetected().getValue() || watson.getIsDetected().getValue());
  }

  public BooleanSupplier botFullAlgae() {
    return () -> algaeFlag;
  }

  public TalonFX getLoaderMotor() {
    return loader;
  }

  public TalonFX getThumbMotor() {
    return thumb;
  }

  public void configThumb() {
    thumb.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Audio.AllowMusicDurDisable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.CurrentLimits.SupplyCurrentLimit = 80;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    config.CurrentLimits.SupplyCurrentLowerTime = 1;

    thumb.getConfigurator().apply(config);
  }

  public void configLoader() {
    loader.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Audio.AllowMusicDurDisable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

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
    config.ProximityParams.ProximityThreshold = 0.05;

    sherlock.getConfigurator().apply(config);
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

    var command = getCurrentCommand();
    if(command != null) {
      Logger.recordOutput("Soda/Dr. Current Command", command.getName());
    }

    if(Constants.DEBUG) {
      SmartDashboard.putNumber("Loader %", loader.get());
    }

    if(!algaeFlag) {
      algaeFlag = isAlgae.getAsBoolean() && loader.getStatorCurrent().getValueAsDouble() > 80;
    }
  }
}
