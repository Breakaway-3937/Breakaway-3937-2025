// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.MrPibb;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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

public class MrPibb extends SubsystemBase {
  private final TalonFX wrist, turret;
  private final MotionMagicVoltage wristRequest, turretRequest;
  private final GenericEntry wristPosition, turretPosition, currentState;
  private MrPibbStates mrPibbState = MrPibbStates.PROTECT;

  /** Creates a new MrPibb.
   *  @since Ankle is no longer with us.
   */
  public MrPibb() {
    wrist = new TalonFX(Constants.MrPibb.WRIST_CAN_ID);
    turret = new TalonFX(Constants.MrPibb.TURRET_CAN_ID);

    configWrist();
    configTurret();

    wristRequest = new MotionMagicVoltage(0).withEnableFOC(true);
    turretRequest = new MotionMagicVoltage(0).withEnableFOC(true);

    wristPosition = Shuffleboard.getTab("MrPibb").add("Wrist", getWristPosition()).withPosition(0, 0).getEntry();
    turretPosition = Shuffleboard.getTab("MrPibb").add("Turret", getTurretPosition()).withPosition(1, 0).getEntry();
    currentState = Shuffleboard.getTab("MrPibb").add("State", getState()).withPosition(2, 0).getEntry();
  }

  public Command setWrist() {
    return runOnce(() -> wrist.setControl(wristRequest.withPosition(mrPibbState.getWrist())));
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

  public BooleanSupplier turretMoving() {
    return () -> Math.abs(getTurretPosition() - mrPibbState.getTurret()) > 0.35;
  }

  public double getWristPosition(){
    return wrist.getPosition().getValueAsDouble();
  }

  public Command waitUntilWristSafe() {
    return Commands.waitUntil(() -> Math.abs(getWristPosition() - mrPibbState.getWrist()) < 0.75);
  }

  public Command waitUntilTurretSafe() {
    return Commands.waitUntil(() -> Math.abs(getTurretPosition() - mrPibbState.getTurret()) < 0.35);
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

  public void configWrist() {
    wrist.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Audio.AllowMusicDurDisable = true;

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

    config.Audio.AllowMusicDurDisable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

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

  @Override
  public void periodic() {
    wristPosition.setDouble(getWristPosition());
    Logger.recordOutput("MrPibb/Wrist", getWristPosition());

    turretPosition.setDouble(getTurretPosition());
    Logger.recordOutput("MrPibb/Turret", getTurretPosition());

    currentState.setString(getState());
    Logger.recordOutput("MrPibb/MrPibb State", getState());

    if(Constants.DEBUG) {
      SmartDashboard.putBoolean("Is Wrist Safe", Math.abs(getWristPosition() - mrPibbState.getWrist()) < 0.75);
      SmartDashboard.putBoolean("Is Turret Safe", Math.abs(getTurretPosition() - mrPibbState.getTurret()) < 0.35);
    }
  }
}
