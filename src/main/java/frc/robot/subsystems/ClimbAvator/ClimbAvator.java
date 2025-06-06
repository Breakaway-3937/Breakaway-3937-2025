// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ClimbAvator;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
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

public class ClimbAvator extends SubsystemBase {
  private final TalonFX shoulderMotor, boulderMotor, elevatorMotor, detonatorMotor, bilboBagginsTheBack;
  private final Follower followerShoulderRequest;
  private final Follower followerElevatorRequest;
  private final MotionMagicExpoVoltage shoulderRequest, elevatorRequest;
  private final MotionMagicVoltage bilboRequest;
  private final GenericEntry elevatorPosition, shoulderPosition, bilboPosition, currentState;
  private ClimbAvatorStates climbAvatorState = ClimbAvatorStates.PROTECT;

  /** Creates a new ClimbAvator.*/
  public ClimbAvator() {
    shoulderMotor = new TalonFX(Constants.ClimbAvator.SHOULDER_CAN_ID);
    boulderMotor = new TalonFX(Constants.ClimbAvator.BOULDER_CAN_ID);
    elevatorMotor = new TalonFX(Constants.ClimbAvator.ELEVATOR_CAN_ID);
    detonatorMotor = new TalonFX(Constants.ClimbAvator.DETONATOR_CAN_ID);
    bilboBagginsTheBack = new TalonFX(Constants.ClimbAvator.BILBO_BAGGINS_THE_BACK);

    followerShoulderRequest = new Follower(Constants.ClimbAvator.SHOULDER_CAN_ID, true);
    followerElevatorRequest = new Follower(Constants.ClimbAvator.ELEVATOR_CAN_ID, true);

    shoulderRequest = new MotionMagicExpoVoltage(0);
    elevatorRequest = new MotionMagicExpoVoltage(0);
    bilboRequest = new MotionMagicVoltage(0);

    configShoulderMotors();
    configElevatorMotors();
    configBilboBagginsTheBack();

    elevatorPosition = Shuffleboard.getTab("ClimbAvator").add("Elevator", getElevatorMotorPosition()).withPosition(0, 0).getEntry();
    shoulderPosition = Shuffleboard.getTab("ClimbAvator").add("Shoulder", getShoulderMotorPosition()).withPosition(1, 0).getEntry();
    bilboPosition = Shuffleboard.getTab("ClimbAvator").add("Bilbo", getBilboMotorPosition()).withPosition(2, 0).getEntry();
    currentState = Shuffleboard.getTab("ClimbAvator").add("State", getState().name()).withPosition(3, 0).getEntry();
  }

  public Command setElevator() {
    return runOnce(() -> elevatorMotor.setControl(elevatorRequest.withPosition(climbAvatorState.getHeight())));
  }

  public Command stopElevator() {
    return runOnce(() -> elevatorMotor.stopMotor());
  }

  public Command setShoulder() {
    return runOnce(() -> shoulderMotor.setControl(shoulderRequest.withPosition(climbAvatorState.getAngle())));
  }

  public Command stopShoulder() {
    return runOnce(() -> shoulderMotor.stopMotor());
  }

  public Command bilboBagginsTheBackForward() {
    return runOnce(() -> bilboBagginsTheBack.set(1));
  }

  public Command bilboBagginsTheBackBackward() {
    return runOnce(() -> bilboBagginsTheBack.set(-1));
  }

  public Command bilboBagginsTheBackStop() {
    return runOnce(() -> bilboBagginsTheBack.set(0));
  }

  public Command preStageBilboBagginsTheBack() {
    return runOnce(() -> bilboBagginsTheBack.setControl(bilboRequest.withPosition(3.79)));
  }

  public double getShoulderMotorPosition() {
    return shoulderMotor.getPosition().getValueAsDouble();
  }

  public double getElevatorMotorPosition() {
    return elevatorMotor.getPosition().getValueAsDouble();
  }

  public double getBilboMotorPosition() {
    return bilboBagginsTheBack.getPosition().getValueAsDouble();
  }

  public ClimbAvatorStates getState() {
    return climbAvatorState;
  }

  public Command waitUntilShoulderSafe() { 
    return Commands.waitUntil(() -> Math.abs(getShoulderMotorPosition() - climbAvatorState.getAngle()) < 2);
  }

  public Command waitUntilElevatorSafe() {
    return Commands.waitUntil(() -> Math.abs(getElevatorMotorPosition() - climbAvatorState.getHeight()) < 0.75);
  }

  public TalonFX getShoulderMotor() {
    return shoulderMotor;
  }

  public TalonFX getBoulderMotor() {
    return boulderMotor;
  }

  public TalonFX getElevatorMotor() {
    return elevatorMotor;
  }

  public TalonFX getDetonatorMotor() {
    return detonatorMotor;
  }

  public TalonFX getBilboBagginsTheBackMotor() {
    return bilboBagginsTheBack;
  }

  public void setClimbAvatorState(ClimbAvatorStates climbAvatorState) {
    this.climbAvatorState = climbAvatorState;
  }

  public void setShoulderBrakeMode() {
    MotorOutputConfigs config = new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive).withNeutralMode(NeutralModeValue.Brake);
    shoulderMotor.getConfigurator().apply(config);
    boulderMotor.getConfigurator().apply(config);
  }

  public void configShoulderMotors() {
    shoulderMotor.getConfigurator().apply(new TalonFXConfiguration());
    boulderMotor.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Audio.AllowMusicDurDisable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Slot0.kS = 0.4;
    config.Slot0.kV = 0.12;
    config.Slot0.kA = 0.01;
    config.Slot0.kP = 8;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;

    config.MotionMagic.MotionMagicExpo_kV = 0.03;
    config.MotionMagic.MotionMagicExpo_kA = 0.025;

    config.CurrentLimits.SupplyCurrentLimit = 80;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    config.CurrentLimits.SupplyCurrentLowerTime = 1;

    shoulderMotor.getConfigurator().apply(config);
    boulderMotor.getConfigurator().apply(config);

    shoulderMotor.setPosition(0);
    boulderMotor.setPosition(0);

    boulderMotor.setControl(followerShoulderRequest);
  }

  public Command configShoulderFast() {
    MotionMagicConfigs config = new MotionMagicConfigs();
    config.MotionMagicCruiseVelocity = 0;
    config.MotionMagicExpo_kV = 0.03;
    config.MotionMagicExpo_kA = 0.025;
    return runOnce(() -> {
      shoulderMotor.getConfigurator().apply(config);
      boulderMotor.getConfigurator().apply(config);
    });
  }

  public Command configShoulderSlow() {
    MotionMagicConfigs config = new MotionMagicConfigs();
    config.MotionMagicCruiseVelocity = 0;
    config.MotionMagicExpo_kV = 0.3;
    config.MotionMagicExpo_kA = 0.05;
    return runOnce(() -> {
      shoulderMotor.getConfigurator().apply(config);
      boulderMotor.getConfigurator().apply(config);
    });
  }

  public void configElevatorMotors() {
    elevatorMotor.getConfigurator().apply(new TalonFXConfiguration());
    detonatorMotor.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Audio.AllowMusicDurDisable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Slot0.kS = 0.25;
    config.Slot0.kV = 0.12;
    config.Slot0.kA = 0.01;
    config.Slot0.kP = 4.8;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0.1;

    config.MotionMagic.MotionMagicExpo_kV = 0.01;
    config.MotionMagic.MotionMagicExpo_kA = 0.0125;

    config.CurrentLimits.SupplyCurrentLimit = 80;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    config.CurrentLimits.SupplyCurrentLowerTime = 1;

    elevatorMotor.getConfigurator().apply(config);
    detonatorMotor.getConfigurator().apply(config);

    elevatorMotor.setPosition(0);
    detonatorMotor.setPosition(0);

    detonatorMotor.setControl(followerElevatorRequest);
  }

  public void configBilboBagginsTheBack() {
    bilboBagginsTheBack.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Audio.AllowMusicDurDisable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.Slot0.kS = 0.25; 
    config.Slot0.kV = 0.12; 
    config.Slot0.kA = 0.01; 
    config.Slot0.kP = 4.8; 
    config.Slot0.kI = 0; 
    config.Slot0.kD = 0; 

    config.MotionMagic.MotionMagicAcceleration = 380;
    config.MotionMagic.MotionMagicCruiseVelocity = 445;
    config.MotionMagic.MotionMagicJerk = 1600;

    config.CurrentLimits.SupplyCurrentLimit = 80;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    config.CurrentLimits.SupplyCurrentLowerTime = 1;

    bilboBagginsTheBack.getConfigurator().apply(config);

    bilboBagginsTheBack.setPosition(0);

  }

  @Override
  public void periodic() {
    elevatorPosition.setDouble(getElevatorMotorPosition());
    Logger.recordOutput("ClimbAvator/Elevator", getElevatorMotorPosition());

    shoulderPosition.setDouble(getShoulderMotorPosition());
    Logger.recordOutput("ClimbAvator/Shoulder", getShoulderMotorPosition());

    bilboPosition.setDouble(getBilboMotorPosition());
    Logger.recordOutput("ClimbAvator/Bilbo", getBilboMotorPosition());
    
    currentState.setString(getState().name());
    Logger.recordOutput("ClimbAvator/ClimbAvator State", getState().name());

    if(Constants.DEBUG) {
      SmartDashboard.putBoolean("Is Shoulder Safe", Math.abs(getShoulderMotorPosition() - climbAvatorState.getAngle()) < 2);
      SmartDashboard.putBoolean("Is Elevator Safe",  Math.abs(getElevatorMotorPosition() - climbAvatorState.getHeight()) < 0.75);
    }

  }
}
