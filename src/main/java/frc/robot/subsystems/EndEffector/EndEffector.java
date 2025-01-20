// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndEffector extends SubsystemBase {
  private final TalonFX wrist;
  private final TalonSRX loader;
  private final MotionMagicExpoVoltage wristRequest;
  private final GenericEntry wristPosition;
  private EndEffectorStates endEffectorState;

  /** Creates a new EndEffector. */
  public EndEffector() {
    wrist = new TalonFX(Constants.EndEffector.WRIST_CAN_ID);
    loader = new TalonSRX(Constants.EndEffector.LOADER_CAN_ID);

    configLoader();
    configWrist();

    wristRequest = new MotionMagicExpoVoltage(0).withEnableFOC(true);

    wristPosition = Shuffleboard.getTab("EndEffector").add("Wrist", getWristPosition()).withPosition(0, 0).getEntry();
  }

  public Command runWrist() {
    return runOnce(() -> wrist.setControl(wristRequest.withPosition(endEffectorState.getAngle())));
  }

  public Command stopWrist() {
    return runOnce(() -> wrist.stopMotor());
  }

  public Command runLoader() {
    return runOnce(() -> loader.set(ControlMode.PercentOutput, endEffectorState.getSpeed()));
  }

  public Command stopLoader() {
    return runOnce(() -> loader.set(ControlMode.PercentOutput, 0));
  }

  public double getSpeed() {
    return loader.getMotorOutputPercent();
  }

  public double getWristPosition(){
    return wrist.getPosition().getValueAsDouble();
  }

  public TalonFX getWristMotor() {
    return wrist;
  }

  public void setEndEffectorState(EndEffectorStates endEffectorState) {
    this.endEffectorState = endEffectorState;
  }

  public void configLoader() {
    loader.configFactoryDefault();

    TalonSRXConfiguration config = new TalonSRXConfiguration();

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

    config.Slot0.kS = 0.4; // Add 0.4 V output to overcome static friction
    config.Slot0.kV = 0.13; // A velocity target of 1 rps results in 0.13 V output
    config.Slot0.kA = 0.1; // An acceleration of 1 rps/s requires 0.1 V output
    config.Slot0.kP = 0; // An error of 1 rps results in 0.2 V output
    config.Slot0.kI = 0; // no output for integrated error
    config.Slot0.kD = 0; // no output for error derivative

    config.CurrentLimits.SupplyCurrentLimit = 70;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    config.CurrentLimits.SupplyCurrentLowerTime = 1;

    config.MotionMagic.MotionMagicAcceleration = 2500;
    config.MotionMagic.MotionMagicCruiseVelocity = 5000;

    wrist.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
   wrist.set(getWristPosition());
  }

}