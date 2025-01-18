// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.BeaterBar;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BeaterBar extends SubsystemBase {
  private final TalonFX ankle, intake; 
  private final MotionMagicExpoVoltage magicRequest;
  private GenericEntry anklePos; 

  /** Creates a new BeaterBar. */
  public BeaterBar() {
    ankle = new TalonFX(Constants.BeaterBar.ANKLE_CAN_ID);
    intake = new TalonFX(Constants.BeaterBar.INTAKE_CAN_ID);

    configIntake();
    configAnkle();

    magicRequest = new MotionMagicExpoVoltage(0).withEnableFOC(true);
  }

  private void runAnkle(double position) {
    ankle.setControl(magicRequest.withPosition(position));
  }

  public Command runAnkle() {
    return runOnce(() -> runAnkle(0));
  }

  public Command stopAnkle() {
    return runOnce(() -> ankle.stopMotor());
  }

  public Command runIntake() {
    return runOnce(() -> intake.set(0));
  }

  public Command stopIntake() {
    return runOnce(() -> intake.stopMotor());
  }

  public double getAnklePosition() {
    return ankle.getPosition().getValueAsDouble();
  }

  public double getSpeed() {
    return intake.get();
  }

  public TalonFX getAnkleMotor() {
    return ankle;
  }

  public TalonFX getIntakeMotor() {
    return intake;
  }

  public void configAnkle() {
    ankle.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.Slot0.kS = 0.4; // Add 0.4 V output to overcome static friction
    config.Slot0.kV = 0.13; // A velocity target of 1 rps results in 0.13 V output
    config.Slot0.kA = 0.1; // An acceleration of 1 rps/s requires 0.1 V output
    config.Slot0.kP = 0; // An error of 1 rps results in 0.2 V output
    config.Slot0.kI = 0; // no output for integrated error
    config.Slot0.kD = 0; // no output for error derivative

    config.CurrentLimits.SupplyCurrentLimit = 35;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 50;
    config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

    config.MotionMagic.MotionMagicAcceleration = 2500;
    config.MotionMagic.MotionMagicCruiseVelocity = 5000;

    ankle.getConfigurator().apply(config);
  }

  public void configIntake() {
    intake.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.CurrentLimits.SupplyCurrentLimit = 35;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = 50;
    config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

    intake.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
