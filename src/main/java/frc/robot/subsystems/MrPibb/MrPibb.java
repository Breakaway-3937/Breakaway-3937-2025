// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.MrPibb;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MrPibb extends SubsystemBase {
  private final TalonFX wrist, turret, loader;
  private final TalonSRX thumb;
  private final MotionMagicVoltage wristRequest, turretRequest;
  private final GenericEntry wristPosition, turretPosition, currentState, coralFull, algaeFull;
  private MrPibbStates mrPibbState = MrPibbStates.PROTECT;

  /** Creates a new MrPibb.
   *  @since Ankle is no longer with us.
   */
  public MrPibb() {
    wrist = new TalonFX(Constants.MrPibb.WRIST_CAN_ID);
    turret = new TalonFX(Constants.MrPibb.TURRET_CAN_ID);
    loader = new TalonFX(Constants.MrPibb.LOADER_CAN_ID);
    thumb = new TalonSRX(Constants.MrPibb.THUMB_CAN_ID);

    configWrist();
    configTurret();
    configLoader();
    configThumb();

    wristRequest = new MotionMagicVoltage(0).withEnableFOC(true);
    turretRequest = new MotionMagicVoltage(0).withEnableFOC(true);

    wristPosition = Shuffleboard.getTab("MrPibb").add("Wrist", getWristPosition()).withPosition(0, 0).getEntry();
    turretPosition = Shuffleboard.getTab("MrPibb").add("Turret", getTurretPosition()).withPosition(1, 0).getEntry();
    currentState = Shuffleboard.getTab("MrPibb").add("State", getState()).withPosition(2, 0).getEntry();
    coralFull = Shuffleboard.getTab("MrPibb").add("Coral Full", botFullCoral().getAsBoolean()).withPosition(3, 0).getEntry();
    algaeFull = Shuffleboard.getTab("MrPibb").add("Algae Full", botFullAlgae().getAsBoolean()).withPosition(4, 0).getEntry();

  }

  public Command setWrist() {
    return runOnce(() -> wrist.setControl(wristRequest.withPosition(mrPibbState.getWrist())));
  }

  public Command setWristNeutral() {
    return runOnce(() -> wrist.setControl(wristRequest.withPosition(MrPibbStates.getNeutralWrist())));
  }

  public Command stopWrist() {
    return runOnce(() -> wrist.stopMotor());
  }

  public Command setTurret() {
    return runOnce(() -> turret.setControl(turretRequest.withPosition(mrPibbState.getTurret()))).unless(botFullAlgae());
  }

  public Command stopTurret() {
    return runOnce(() -> turret.stopMotor());
  }

  public Command runLoader() {
    return runOnce(() -> loader.set(1));
  }

  public Command runLoaderReverse() {
    return runOnce(() -> loader.set(-1));
  }

  public Command stopLoader() {
    return runOnce(() -> loader.stopMotor());
  }

  public Command runThumbForward() {
    return runOnce(() -> thumb.set(ControlMode.PercentOutput, 1));
  }

  public Command runThumbBackward() {
    return runOnce(() -> thumb.set(ControlMode.PercentOutput, -1));
  }

  public Command stopThumb() {
    return runOnce(() -> thumb.set(ControlMode.PercentOutput, 0));
  }

  public Command runUntilFull() {
    return runLoader().andThen(Commands.waitUntil(botFullCoral())).andThen(stopLoader());
  }

  //TODO: Write this method.
  public BooleanSupplier botFullCoral() {
    return () -> false;
  }

  //TODO: Write this method.
  public BooleanSupplier botFullAlgae() {
    return () -> false;
  } 

  public BooleanSupplier wristForward() {
    return () -> mrPibbState.getWrist() >= getWristPosition();
  }

  public BooleanSupplier wristPositive() {
    return () -> mrPibbState.getWrist() >= 0;
  }

  public double getWristPosition(){
    return wrist.getPosition().getValueAsDouble();
  }

  public Command waitUntilWristSafe() {
    return Commands.waitUntil(() -> Math.abs(getWristPosition() - mrPibbState.getWrist()) < 0.75);
  }

  public Command waitUntilWristNeutralSafe() {
    return Commands.waitUntil(() -> Math.abs(getWristPosition() - MrPibbStates.getNeutralWrist()) < 0.75);
  }

  public Command waitUntilTurretSafe() {
    return Commands.waitUntil(() -> Math.abs(getTurretPosition() - mrPibbState.getTurret()) < 0.75);
  }

  public TalonFX getWristMotor() {
    return wrist;
  }

  public double getTurretPosition(){
    return turret.getPosition().getValueAsDouble();
  }

  public String getState() {
    return mrPibbState.name();
  }

  public TalonFX getTurretMotor() {
    return turret;
  }

  public void setMrPibbState(MrPibbStates mrPibbState) {
    this.mrPibbState = mrPibbState;
  }

  public void configThumb() {
    thumb.configFactoryDefault();

    TalonSRXConfiguration config = new TalonSRXConfiguration();

    config.peakCurrentDuration = 100;
    config.peakCurrentLimit = 50;
    config.continuousCurrentLimit = 35;

    thumb.setInverted(true);

    thumb.configAllSettings(config);
    thumb.enableCurrentLimit(true);
  }

  public void configWrist() {
    wrist.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Slot0.kS = 0.25;
    config.Slot0.kV = 0.12;
    config.Slot0.kA = 0.01;
    config.Slot0.kP = 4.45;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0.18;

    config.CurrentLimits.SupplyCurrentLimit = 80;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    config.CurrentLimits.SupplyCurrentLowerTime = 1;

    config.MotionMagic.MotionMagicAcceleration = 175;
    config.MotionMagic.MotionMagicCruiseVelocity = 200;
    config.MotionMagic.MotionMagicJerk = 1600;

    wrist.getConfigurator().apply(config);
    wrist.setPosition(0);
  }

  public void configTurret() {
    turret.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.Slot0.kS = 0.25;
    config.Slot0.kV = 0.12;
    config.Slot0.kA = 0.01;
    config.Slot0.kP = 6;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0.24;

    config.CurrentLimits.SupplyCurrentLimit = 80;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    config.CurrentLimits.SupplyCurrentLowerTime = 1;

    config.MotionMagic.MotionMagicAcceleration = 155;
    config.MotionMagic.MotionMagicCruiseVelocity = 170;
    config.MotionMagic.MotionMagicJerk = 1600;

    turret.getConfigurator().apply(config);
    turret.setPosition(0);
  }

  public void configLoader() {
    loader.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.CurrentLimits.SupplyCurrentLimit = 80;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    config.CurrentLimits.SupplyCurrentLowerTime = 1;

    loader.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
    wristPosition.setDouble(getWristPosition());
    Logger.recordOutput("MrPibb/Wrist", getWristPosition());

    turretPosition.setDouble(getTurretPosition());
    Logger.recordOutput("MrPibb/Turret", getTurretPosition());

    currentState.setString(getState());
    Logger.recordOutput("MrPibb/MrPibb State", getState());

    coralFull.setBoolean(botFullCoral().getAsBoolean());
    Logger.recordOutput("MrPibb/Coral Full", botFullCoral().getAsBoolean());

    algaeFull.setBoolean(botFullAlgae().getAsBoolean());
    Logger.recordOutput("MrPibb/Algae Full", botFullAlgae().getAsBoolean());
  }
}
