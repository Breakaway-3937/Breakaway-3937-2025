// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ClimbAvator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbAvator extends SubsystemBase {
  private final TalonFX shoulderMotor, shoulderFollowerMotor, elevatorMotor, elevatorFollowerMotor;
  private final Follower followerShoulderRequest = new Follower(Constants.ClimbAvator.SHOULDER_CAN_ID, false);//FIXME Check direction
  private final Follower followerElevatorRequest = new Follower(Constants.ClimbAvator.ELEVATOR_CAN_ID, false);//FIXME Check direction
  private final MotionMagicExpoTorqueCurrentFOC shoulderRequest = new MotionMagicExpoTorqueCurrentFOC(0);
  private final MotionMagicExpoTorqueCurrentFOC elevatorRequest = new MotionMagicExpoTorqueCurrentFOC(0);
  private final GenericEntry elevatorPosition, shoulderPosition;
  private ClimbAvatorStates climbAvatorState;

  /** Creates a new ClimbAvator.*/
  public ClimbAvator() {
    shoulderMotor = new TalonFX(Constants.ClimbAvator.SHOULDER_CAN_ID);
    shoulderFollowerMotor = new TalonFX(Constants.ClimbAvator.SHOULDER_FOLLOWER_CAN_ID);
    elevatorMotor = new TalonFX(Constants.ClimbAvator.ELEVATOR_CAN_ID);
    elevatorFollowerMotor = new TalonFX(Constants.ClimbAvator.ELEVATOR_FOLLOWER_CAN_ID);

    configShoulderMotors();
    configElevatorMotors();

    elevatorPosition = Shuffleboard.getTab("ClimbAvator").add("Elevator", getElevatorMotorPosition()).withPosition(0, 0).getEntry();
    shoulderPosition = Shuffleboard.getTab("ClimbAvator").add("Climber", getShoulderMotorPosition()).withPosition(0, 1).getEntry();
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

  public Command stopShoulder(){
    return runOnce(() -> shoulderMotor.stopMotor());
  }

  public double getShoulderMotorPosition() {
    return shoulderMotor.getPosition().getValueAsDouble();
  }

  public double getElevatorMotorPosition() {
    return elevatorMotor.getPosition().getValueAsDouble();
  }

  public TalonFX getShoulderMotor() {
    return shoulderMotor;
  }

  public TalonFX getShoulderFollowerMotor() {
    return shoulderFollowerMotor;
  }

  public TalonFX getElevatorMotor() {
    return elevatorMotor;
  }

  public TalonFX getElevatorFollowerMotor() {
    return elevatorFollowerMotor;
  }

    public void setClimbAvatorState(ClimbAvatorStates climbAvatorState) {
    this.climbAvatorState = climbAvatorState;
  }

  public void configShoulderMotors() {
    shoulderMotor.getConfigurator().apply(new TalonFXConfiguration());
    shoulderFollowerMotor.getConfigurator().apply(new TalonFXConfiguration());

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

    shoulderMotor.getConfigurator().apply(config);
    shoulderFollowerMotor.getConfigurator().apply(config);

    shoulderFollowerMotor.setControl(followerShoulderRequest);
  }

  public void configElevatorMotors() {
    elevatorMotor.getConfigurator().apply(new TalonFXConfiguration());
    elevatorFollowerMotor.getConfigurator().apply(new TalonFXConfiguration());

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

    elevatorMotor.getConfigurator().apply(config);
    elevatorFollowerMotor.getConfigurator().apply(config);

    elevatorFollowerMotor.setControl(followerElevatorRequest);
  }
  @Override
  public void periodic() {
    elevatorPosition.setDouble(getElevatorMotorPosition());
    shoulderPosition.setDouble(getShoulderMotorPosition());
  }
}
