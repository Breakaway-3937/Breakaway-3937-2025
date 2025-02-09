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
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.hal.util.BoundaryException;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MrPibb extends SubsystemBase {
  private final TalonFX wrist, turret;
  private final TalonSRX loader, thumb;
  private final MotionMagicVoltage wristRequest, turretRequest;
  private final GenericEntry wristPosition, turretPosition;
  private MrPibbStates mrPibbState;
  private double maxValue = -0.197200, minValue = 0; //TODO

  /** Creates a new MrPibb.
   *  @since Ankle is no longer with us.
   */
  public MrPibb() {
    wrist = new TalonFX(Constants.MrPibb.WRIST_CAN_ID);
    turret = new TalonFX(Constants.MrPibb.TURRET_CAN_ID);
    loader = new TalonSRX(Constants.MrPibb.LOADER_CAN_ID);
    thumb = new TalonSRX(Constants.MrPibb.THUMB_CAN_ID);

    configLoader();
    configWrist();
    configTurret();

    wristRequest = new MotionMagicVoltage(0).withEnableFOC(true);
    turretRequest = new MotionMagicVoltage(0).withEnableFOC(true);

    wristPosition = Shuffleboard.getTab("MrPibb").add("Wrist", getWristPosition()).withPosition(0, 0).getEntry();
    turretPosition = Shuffleboard.getTab("MrPibb").add("Turret", getTurretPosition()).withPosition(0, 0).getEntry();
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
    return runOnce(() -> turret.setControl(turretRequest.withPosition(mrPibbState.getTurret())));
  }

  public Command stopTurret() {
    return runOnce(() -> turret.stopMotor());
  }

  public Command runLoader() {
    return runOnce(() -> loader.set(ControlMode.PercentOutput, mrPibbState.getSpeed()));
  }

  public Command stopLoader() {
    return runOnce(() -> loader.set(ControlMode.PercentOutput, 0));
  }

  public Command runThumb() {
    return runOnce(() -> thumb.set(ControlMode.PercentOutput, 0));
  }

  public Command stopThumb() {
    return runOnce(() -> thumb.set(ControlMode.PercentOutput, 0));
  }

  public Command runUntilFull() {
    return runLoader().andThen(Commands.waitUntil(botFull())).andThen(stopLoader());
  }

  //TODO: Write this method.
  public BooleanSupplier botFull() {
    return () -> false;
  }

  public double getSpeed() {
    return loader.getMotorOutputPercent();
  }

  public double getWristPosition(){
    return wrist.getPosition().getValueAsDouble();
  }

  public Command waitUntilWristSafe() {

    return Commands.waitUntil(() -> {
      if(getWristPosition() < 11.2 && getWristPosition() > 6.35) {
        return true;
      }
      else {
        return false;
      }
    }).alongWith(new PrintCommand("Wrist unsafe"));
  }

  public Command waitUntilTurretSafe() {
    return Commands.waitUntil(() -> MathUtil.isNear(mrPibbState.getTurret(), getTurretPosition(), 0.2)).alongWith(new PrintCommand("turret unsafe"));
  }

  public TalonFX getWristMotor() {
    return wrist;
  }

  public double getTurretPosition(){
    return turret.getPosition().getValueAsDouble();
  }

  public TalonFX getTurretMotor() {
    return turret;
  }

  public void setMrPibbState(MrPibbStates mrPibbState) {
    this.mrPibbState = mrPibbState;
  }

  public void configLoader() {
    loader.configFactoryDefault();

    TalonSRXConfiguration config = new TalonSRXConfiguration();

    //TODO: Tune these values.
    config.peakCurrentDuration = 100;
    config.peakCurrentLimit = 40;
    config.continuousCurrentLimit = 30;

    loader.configAllSettings(config);
    loader.enableCurrentLimit(true);
  }

  public void configWrist() {
    wrist.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    //TODO: Tune these values.
    config.Slot0.kS = 0.25;
    config.Slot0.kV = 0.12;
    config.Slot0.kA = 0.01;
    config.Slot0.kP = 4.45;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0.18;

    /*config.CurrentLimits.SupplyCurrentLimit = 70;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    config.CurrentLimits.SupplyCurrentLowerTime = 1;*/

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

    //TODO: Tune these values.
    config.Slot0.kS = 0.25;
    config.Slot0.kV = 0.12;
    config.Slot0.kA = 0.01;
    config.Slot0.kP = 2.89;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0.24;

    /*config.CurrentLimits.SupplyCurrentLimit = 70;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    config.CurrentLimits.SupplyCurrentLowerTime = 1;*/

    config.MotionMagic.MotionMagicAcceleration = 155;
    config.MotionMagic.MotionMagicCruiseVelocity = 170;
    config.MotionMagic.MotionMagicJerk = 1600;

    turret.getConfigurator().apply(config);
    turret.setPosition(0);
  }

  @Override
  public void periodic() {
    wristPosition.setDouble(getWristPosition());
    Logger.recordOutput("Wrist", getWristPosition());
    turretPosition.setDouble(getTurretPosition());
    Logger.recordOutput("Turret", getTurretPosition());
  }
}
