// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ClimbAvator.ClimbAvator;
import frc.robot.subsystems.ClimbAvator.ClimbAvatorStates;
import frc.robot.subsystems.MrPibb.MrPibb;
import frc.robot.subsystems.MrPibb.MrPibbStates;

public class SuperSubsystem extends SubsystemBase {
  private final ClimbAvator s_ClimbAvator;
  private final MrPibb s_MrPibb;

  /** Creates a new SuperSubsystem. */
  public SuperSubsystem(ClimbAvator s_ClimbAvator, MrPibb s_MrPibb) {
    this.s_ClimbAvator = s_ClimbAvator; 
    this.s_MrPibb = s_MrPibb;
  }

  public Command load() {
    loadState();
    //TODO: Add safety check and actual code. Look at MrPibb.java for example.
    return Commands.none();
  }

  public Command runToProtect() {
    protectState();
    
    if(s_ClimbAvator.isShoulderPrimeLocation()) {
      return handleRetractAndExtend();
    }
    else {
      return handleRetractAndExtendFromGround();
    }
  }

  public Command runToL1() {
    level1State();
    
    if(s_ClimbAvator.isShoulderPrimeLocation()) {
      return handleRetractAndExtend();
    }
    else {
      return handleRetractAndExtendFromGround();
    }
  }

  public Command runToL2() {
    level2State();
    
    if(s_ClimbAvator.isShoulderPrimeLocation()) {
      return handleRetractAndExtend();
    }
    else {
      return handleRetractAndExtendFromGround();
    }
  }

  public Command runToL3() {
    level3State();
    
    if(s_ClimbAvator.isShoulderPrimeLocation()) {
      return handleRetractAndExtend();
    }
    else {
      return handleRetractAndExtendFromGround();
    }
  }

  public Command runToStation() {
    loadState();
    
    if(s_ClimbAvator.isShoulderPrimeLocation()) {
      return handleRetractAndExtend();
    }
    else {
      return handleRetractAndExtendFromGround();
    }
  }

  public Command runToPreStage() {
    preStageState();
    
    if(s_ClimbAvator.isShoulderPrimeLocation()) {
      return handleRetractAndExtend();
    }
    else {
      return handleRetractAndExtendFromGround();
    }
  }

  public Command runtToZero() {
    zeroState();
    
    if(s_ClimbAvator.isShoulderPrimeLocation()) {
      return handleRetractAndExtend();
    }
    else {
      return handleRetractAndExtendFromGround();
    }
  }

  public Command runToBarge() {
    bargeState();

    if(s_ClimbAvator.isShoulderPrimeLocation()) {
      return handleRetractAndExtend();
    }
    else {
      return handleRetractAndExtendFromGround();
    }
  }

  public Command handleRetractAndExtend() {
    return s_MrPibb.setWristNeutral().andThen(s_MrPibb.waitUntilWristSafe())
                              .andThen(s_MrPibb.setTurret()).andThen(s_MrPibb.waitUntilTurretSafe())
                              .andThen(s_ClimbAvator.setElevatorNeutral()).andThen(s_ClimbAvator.waitUntilElevatorSafe())
                              .andThen(s_ClimbAvator.setShoulder()).andThen(s_ClimbAvator.waitUntilShoulderSafe())
                              .andThen(s_ClimbAvator.setElevator()).andThen(s_ClimbAvator.waitUntilElevatorSafe())
                              .andThen(s_MrPibb.setWrist()).andThen(s_MrPibb.waitUntilWristSafe());
  }

  public Command handleRetractAndExtendFromGround() {
    return s_ClimbAvator.setShoulder().andThen(s_ClimbAvator.waitUntilShoulderSafe())
                        .andThen(s_MrPibb.setWrist()).andThen(s_MrPibb.waitUntilWristSafe())
                        .andThen(s_MrPibb.setTurret()).andThen(s_MrPibb.waitUntilTurretSafe())
                        .andThen(s_ClimbAvator.setElevator()).andThen(s_ClimbAvator.waitUntilElevatorSafe());
  }

  public void loadState() {
    s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.STATION);
    s_MrPibb.setMrPibbState(MrPibbStates.STATION);
  }

  public void protectState() {
    s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.PROTECT);
    s_MrPibb.setMrPibbState(MrPibbStates.PROTECT);
  }

  public void level1State() {
    s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.L1);
    s_MrPibb.setMrPibbState(MrPibbStates.L1);
  }

  public void level2State() {
    s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.L2);
    s_MrPibb.setMrPibbState(MrPibbStates.L2);
  }

  public void level3State() {
    s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.L3);
    s_MrPibb.setMrPibbState(MrPibbStates.L3);
  }

  public void bargeState() {
    s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.BARGE);
    s_MrPibb.setMrPibbState(MrPibbStates.BARGE);
  }

  public void zeroState() {
    s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.CLIMB);
    s_MrPibb.setMrPibbState(MrPibbStates.CLIMB);
  }

  public void preStageState() {
    s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.CORAL_PRESTAGE);
    s_MrPibb.setMrPibbState(MrPibbStates.CORAL_PRESTAGE);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
