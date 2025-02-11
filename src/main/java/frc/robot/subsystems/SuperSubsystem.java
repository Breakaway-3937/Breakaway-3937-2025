// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
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

  public Command runSubsystems() {
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

  //TODO TEST ME
  public Command loadState() {
    return runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.STATION))
               .alongWith(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.STATION)))
               .andThen(runSubsystems());
  }

  public Command climbState() {
    return runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.CLIMB_TEST))
               .alongWith(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.CLIMB_TEST)));
  }

  public Command lowerAlgeaState() {
    return runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.LOWER_ALGEA))
               .alongWith(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.LOWER_ALGEA)));
  }

  public Command upperAlgeaState() {
    return runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.UPPER_ALGEA))
               .alongWith(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.UPPER_ALGEA)));
  }

  public Command protectState() {
    return runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.PROTECT))
               .alongWith(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.PROTECT)));
  }

  public Command level1State() {
    return runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.L1))
               .alongWith(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.L1)));
  }

  public Command level2State() {
    return runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.L2))
               .alongWith(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.L2)));
  }

  public Command level3State() {
    return runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.L3))
               .alongWith(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.L3)));
  }

  public Command level4State() {
    return runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.L4))
               .alongWith(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.L4)));
  }

  public Command bargeState() {
    return runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.BARGE))
               .alongWith(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.BARGE)));
  }

  public Command zeroState() {
    return runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.CLIMB))
               .alongWith(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.CLIMB)));
  }

  public Command preStageState() {
    return runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.CORAL_PRESTAGE))
               .alongWith(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.CORAL_PRESTAGE)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
