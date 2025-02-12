// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

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

  public Command runSubsystems() {
    return s_MrPibb.setWristNeutral().andThen(s_MrPibb.waitUntilWristSafe())
                                     .andThen(s_MrPibb.setTurretNeutral()).andThen(s_MrPibb.waitUntilTurretSafe())
                                     .andThen(s_ClimbAvator.setElevatorNeutral()).andThen(s_ClimbAvator.waitUntilElevatorSafe())
                                     .andThen(s_ClimbAvator.setShoulder()).andThen(s_ClimbAvator.waitUntilShoulderSafe())
                                     .andThen(s_ClimbAvator.setElevator()).andThen(s_ClimbAvator.waitUntilElevatorSafe())
                                     .andThen(s_MrPibb.setTurret()).andThen(s_MrPibb.waitUntilTurretSafe())
                                     .andThen(s_MrPibb.setWrist()).andThen(s_MrPibb.waitUntilWristSafe()); 
                                     //TODO wrist before turret?
                                     //If wrist moving forward wrist needs to go first if moving back turret needs to go first.
                                     //We need more conditions for example if going to floor we are going to bring elevator and shoudler down then move turret we hit a swerve module i think
  }

  public Command wristOrTurret() {
    return Commands.either(s_MrPibb.setWrist().andThen(s_MrPibb.waitUntilWristSafe())
                                              .andThen(s_MrPibb.setTurret())
                                              .andThen(s_MrPibb.setTurret()),
                           s_MrPibb.setTurret().andThen(s_MrPibb.waitUntilTurretSafe())
                                               .andThen(s_MrPibb.setWrist())
                                               .andThen(s_MrPibb.waitUntilWristSafe()),                    
                           s_MrPibb.wristForward());
  }

  /* Intake States */
  public Command loadState() {
    return runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.STATION))
               .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.STATION)))
               .andThen(runSubsystems());
  }

  public Command preStageState() {
    return runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.CORAL_PRESTAGE))
               .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.CORAL_PRESTAGE)));
  }

  /* Algae States */
  public Command lowerAlgaeState() {
    return runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.LOWER_ALGAE))
               .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.LOWER_ALGAE)));
  }

  public Command upperAlgaeState() {
    return runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.UPPER_ALGAE))
               .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.UPPER_ALGAE)));
  }

  public Command bargeState() {
    return runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.BARGE))
               .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.BARGE)));
  }

  /* Scoring States */
  public Command level1State() {
    return runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.L1))
               .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.L1)));
  }

  public Command level2State() {
    return runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.L2))
               .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.L2)));
  }

  public Command level3State() {
    return runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.L3))
               .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.L3)));
  }

  public Command level4State() {
    return runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.L4))
               .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.L4)));
  }

  /* Other States */
  public Command climbState() {
    return runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.CLIMB_TEST))
               .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.CLIMB_TEST)));
  }

  public Command zeroState() {
    return runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.CLIMB))
               .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.CLIMB)));
  }

  public Command protectState() {
    return runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.PROTECT))
               .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.PROTECT)));
  }

  public BooleanSupplier botFullCoral() {
    return s_MrPibb.botFullCoral();
  }

}
