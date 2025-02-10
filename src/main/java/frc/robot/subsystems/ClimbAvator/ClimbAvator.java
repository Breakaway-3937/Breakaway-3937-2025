// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ClimbAvator;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.MrPibb.MrPibbStates;

public class ClimbAvator extends SubsystemBase {
  private final TalonFX shoulderMotor, boulderMotor, elevatorMotor, detonatorMotor;
  private final Follower followerShoulderRequest;
  private final Follower followerElevatorRequest;
  private final MotionMagicVoltage shoulderRequest;
  private final MotionMagicVoltage elevatorRequest;
  private final GenericEntry elevatorPosition, shoulderPosition, currentState;
  private ClimbAvatorStates climbAvatorState;

  /** Creates a new ClimbAvator.*/
  public ClimbAvator() {
    shoulderMotor = new TalonFX(Constants.ClimbAvator.SHOULDER_CAN_ID);
    boulderMotor = new TalonFX(Constants.ClimbAvator.BOULDER_CAN_ID);
    elevatorMotor = new TalonFX(Constants.ClimbAvator.ELEVATOR_CAN_ID);
    detonatorMotor = new TalonFX(Constants.ClimbAvator.DETONATOR_CAN_ID);

    followerShoulderRequest = new Follower(Constants.ClimbAvator.SHOULDER_CAN_ID, true);//TODO: Check direction.
    followerElevatorRequest = new Follower(Constants.ClimbAvator.ELEVATOR_CAN_ID, true);//TODO: Check direction.

    shoulderRequest = new MotionMagicVoltage(0);
    elevatorRequest = new MotionMagicVoltage(0);

    configShoulderMotors();
    configElevatorMotors();

    elevatorPosition = Shuffleboard.getTab("ClimbAvator").add("Elevator", getElevatorMotorPosition()).withPosition(0, 0).getEntry();
    shoulderPosition = Shuffleboard.getTab("ClimbAvator").add("Climber", getShoulderMotorPosition()).withPosition(1, 0).getEntry();
    currentState = Shuffleboard.getTab("ClimbAvator").add("State", getState()).withPosition(2, 0).getEntry();
  }

  public Command setElevator() {
    return runOnce(() -> elevatorMotor.setControl(elevatorRequest.withPosition(climbAvatorState.getHeight())));
  }

  public Command stopElevator() {
    return runOnce(() -> elevatorMotor.stopMotor());
  }

  public Command setElevatorNeutral() {
    return runOnce(() -> elevatorMotor.setControl(elevatorRequest.withPosition(ClimbAvatorStates.getNeutralElevator())));
  }
  //hello world

  public Command setShoulder() {
    return runOnce(() -> shoulderMotor.setControl(shoulderRequest.withPosition(climbAvatorState.getAngle())));
  }

  public void setShoulderTest() {
    shoulderMotor.setControl(shoulderRequest.withPosition(-0.020996*4));
  }

  public Command stopShoulder(){
    return runOnce(() -> shoulderMotor.stopMotor());
  }

  public boolean isShoulderPrimeLocation() {
    if(getShoulderMotorPosition() <= -0.114 && getShoulderMotorPosition() >= -0.228) {
      return true;
    } 
    else {
      return false;
    }
  }

  public double getShoulderMotorPosition() {
    return shoulderMotor.getPosition().getValueAsDouble();
  }

  public double getElevatorMotorPosition() {
    return elevatorMotor.getPosition().getValueAsDouble();
  }

  public String getState() {
    return climbAvatorState.toString();
  }

  public Command waitUntilShoulderSafe() {
    return Commands.waitUntil(() -> MathUtil.isNear(climbAvatorState.getAngle(), getShoulderMotorPosition(), .75)).alongWith(new PrintCommand("shoulder unsafe"));
  }

  public Command waitUntilElevatorSafe() {
    return Commands.waitUntil(() -> MathUtil.isNear(climbAvatorState.getHeight(), getElevatorMotorPosition(), .75)).alongWith(new PrintCommand("ele unsafe"));
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

    //TODO: Tune these values.
    config.Slot0.kS = 0.25;
    config.Slot0.kV = 0.2;
    config.Slot0.kA = 0.03;
    config.Slot0.kP = 67.2;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;

    /*config.CurrentLimits.SupplyCurrentLimit = 70;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    config.CurrentLimits.SupplyCurrentLowerTime = 1;*/

    config.MotionMagic.MotionMagicAcceleration = 2880;
    config.MotionMagic.MotionMagicCruiseVelocity = 2880;
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

    //TODO: Tune these values.
    config.Slot0.kS = 0.25;
    config.Slot0.kV = 0.12;
    config.Slot0.kA = 0.01;
    config.Slot0.kP = 4.8;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0.1;

    /*config.CurrentLimits.SupplyCurrentLimit = 70;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    config.CurrentLimits.SupplyCurrentLowerTime = 1;*/

    config.MotionMagic.MotionMagicAcceleration = 200;
    config.MotionMagic.MotionMagicCruiseVelocity = 280;
    config.MotionMagic.MotionMagicJerk = 1600;

    elevatorMotor.getConfigurator().apply(config);
    detonatorMotor.getConfigurator().apply(config);

    elevatorMotor.setPosition(0);
    detonatorMotor.setPosition(0);

    detonatorMotor.setControl(followerElevatorRequest);
  }

  @Override
  public void periodic() {
    elevatorPosition.setDouble(getElevatorMotorPosition());
    shoulderPosition.setDouble(getShoulderMotorPosition());
    currentState.setString(getState());
    System.out.println(getShoulderMotorPosition());
    //System.out.println("closed Hello: Hello agian " + shoulderMotor.getClosedLoopReference().getValueAsDouble());
    //System.out.println(shoulderRequest.Position);
    Logger.recordOutput("Elevator", getElevatorMotorPosition());
    Logger.recordOutput("Shoulder", getShoulderMotorPosition());
  }
}
