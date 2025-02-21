// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ClimbAvator;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
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

public class ClimbAvator extends SubsystemBase {
  private final TalonFX shoulderMotor, boulderMotor, elevatorMotor, detonatorMotor;
  private final TalonSRX bagTheBack;
  private final Follower followerShoulderRequest;
  private final Follower followerElevatorRequest;
  private final MotionMagicVoltage shoulderRequest;
  private final MotionMagicExpoVoltage elevatorRequest;
  private final GenericEntry elevatorPosition, shoulderPosition, currentState;
  private ClimbAvatorStates climbAvatorState = ClimbAvatorStates.PROTECT;

  /** Creates a new ClimbAvator.*/
  public ClimbAvator() {
    shoulderMotor = new TalonFX(Constants.ClimbAvator.SHOULDER_CAN_ID);
    boulderMotor = new TalonFX(Constants.ClimbAvator.BOULDER_CAN_ID);
    elevatorMotor = new TalonFX(Constants.ClimbAvator.ELEVATOR_CAN_ID);
    detonatorMotor = new TalonFX(Constants.ClimbAvator.DETONATOR_CAN_ID);
    bagTheBack = new TalonSRX(Constants.ClimbAvator.BAG_THE_BACK);

    followerShoulderRequest = new Follower(Constants.ClimbAvator.SHOULDER_CAN_ID, true);
    followerElevatorRequest = new Follower(Constants.ClimbAvator.ELEVATOR_CAN_ID, true);

    shoulderRequest = new MotionMagicVoltage(0);
    elevatorRequest = new MotionMagicExpoVoltage(0);//.withEnableFOC(true);

    configShoulderMotors();
    configElevatorMotors();

    elevatorPosition = Shuffleboard.getTab("ClimbAvator").add("Elevator", getElevatorMotorPosition()).withPosition(0, 0).getEntry();
    shoulderPosition = Shuffleboard.getTab("ClimbAvator").add("Shoulder", getShoulderMotorPosition()).withPosition(1, 0).getEntry();
    currentState = Shuffleboard.getTab("ClimbAvator").add("State", getState().name()).withPosition(2, 0).getEntry();
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

  public Command bagForward() {
    return runOnce(() -> bagTheBack.set(TalonSRXControlMode.PercentOutput, 1));
  }

  public Command bagBackward() {
    return runOnce(() -> bagTheBack.set(TalonSRXControlMode.PercentOutput, -1));
  }

  public Command bagStop() {
    return runOnce(() -> bagTheBack.set(TalonSRXControlMode.PercentOutput, 0));
  }

  public double getShoulderMotorPosition() {
    return shoulderMotor.getPosition().getValueAsDouble();
  }

  public double getElevatorMotorPosition() {
    return elevatorMotor.getPosition().getValueAsDouble();
  }

  public ClimbAvatorStates getState() {
    return climbAvatorState;
  }

  public Command waitUntilShoulderSafe() { 
    return Commands.waitUntil(() -> Math.abs(getShoulderMotorPosition() - climbAvatorState.getAngle()) < 0.05);
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

  public void setClimbAvatorState(ClimbAvatorStates climbAvatorState) {
    this.climbAvatorState = climbAvatorState;
  }

  public void configShoulderMotors() {
    shoulderMotor.getConfigurator().apply(new TalonFXConfiguration());
    boulderMotor.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.Feedback.SensorToMechanismRatio = 250;

    config.Slot0.kS = 5;
    config.Slot0.kV = 5;
    config.Slot0.kA = 15;
    config.Slot0.kP = 2;//900;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;

    config.MotionMagic.MotionMagicExpo_kV = 0.12; // kV is around 0.12 V/rps
    config.MotionMagic.MotionMagicExpo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s)

    config.CurrentLimits.SupplyCurrentLimit = 80;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    config.CurrentLimits.SupplyCurrentLowerTime = 1;

    config.MotionMagic.MotionMagicCruiseVelocity = 10000;
    config.MotionMagic.MotionMagicAcceleration = 5000;
    config.MotionMagic.MotionMagicJerk = 4000;

    shoulderMotor.getConfigurator().apply(config);
    boulderMotor.getConfigurator().apply(config);

    shoulderMotor.setPosition(0);
    boulderMotor.setPosition(0);

    boulderMotor.setControl(followerShoulderRequest);
  }

  public void configElevatorMotors() {
    elevatorMotor.getConfigurator().apply(new TalonFXConfiguration());
    detonatorMotor.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Slot0.kS = 0.25;
    config.Slot0.kV = 0.12;
    config.Slot0.kA = 0.01;
    config.Slot0.kP = 4.8;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0.1;

    config.CurrentLimits.SupplyCurrentLimit = 80;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    config.CurrentLimits.SupplyCurrentLowerTime = 1;

    config.MotionMagic.MotionMagicAcceleration = 380;
    config.MotionMagic.MotionMagicCruiseVelocity = 445;
    config.MotionMagic.MotionMagicJerk = 1600;

    elevatorMotor.getConfigurator().apply(config);
    detonatorMotor.getConfigurator().apply(config);

    elevatorMotor.setPosition(0);
    detonatorMotor.setPosition(0);

    detonatorMotor.setControl(followerElevatorRequest);
  }

  public void configBag() {
    bagTheBack.configFactoryDefault();

    TalonSRXConfiguration config = new TalonSRXConfiguration();

    config.peakCurrentDuration = 100;
    config.peakCurrentLimit = 50;
    config.continuousCurrentLimit = 35;

    bagTheBack.configAllSettings(config);
    bagTheBack.enableCurrentLimit(true);
  }

  @Override
  public void periodic() {
    elevatorPosition.setDouble(getElevatorMotorPosition());
    Logger.recordOutput("ClimbAvator/Elevator", getElevatorMotorPosition());

    shoulderPosition.setDouble(getShoulderMotorPosition());
    Logger.recordOutput("ClimbAvator/Shoulder", getShoulderMotorPosition());
    
    currentState.setString(getState().name());
    Logger.recordOutput("ClimbAvator/ClimbAvator State", getState().name());
  }
}
