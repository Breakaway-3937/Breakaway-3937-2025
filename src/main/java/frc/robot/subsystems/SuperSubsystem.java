// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.EndEffector.EndEffectorStates;
import frc.robot.subsystems.ClimbAvator.ClimbAvator;
import frc.robot.subsystems.ClimbAvator.ClimbAvatorStates;
import frc.robot.subsystems.BeaterBar.BeaterBar;
import frc.robot.subsystems.BeaterBar.BeaterBarStates;

public class SuperSubsystem extends SubsystemBase {
  private final ClimbAvator s_ClimbAvator;
  private final EndEffector s_EndEffector;
  private final BeaterBar s_BeaterBar; 

  /** Creates a new SuperSubsystem. */
  public SuperSubsystem(ClimbAvator s_ClimbAvator, EndEffector s_EndEffector, BeaterBar s_BeaterBar) {
    this.s_ClimbAvator = s_ClimbAvator; 
    this.s_EndEffector = s_EndEffector;
    this.s_BeaterBar = s_BeaterBar;
  }

  public Command load() {
    loadState();
    //TODO: Add safety check and actual code. Look at EndEffector.java for example.
    return Commands.none();
  }

  public Command runToProtect() {
    protectState();
    //TODO: Add safety check and actual code. Look at EndEffector.java for example.
    return Commands.none();
  }

  public void loadState() {
    s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.STATION);
    s_EndEffector.setEndEffectorState(EndEffectorStates.STATION);
    s_BeaterBar.setBeaterState(BeaterBarStates.PROTECT);
  }

  public void protectState() {
    s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.PROTECT);
    s_EndEffector.setEndEffectorState(EndEffectorStates.PROTECT);
    s_BeaterBar.setBeaterState(BeaterBarStates.PROTECT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
