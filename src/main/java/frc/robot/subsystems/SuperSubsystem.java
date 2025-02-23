// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.Conversions;
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
    return Commands.either(disperse(), condense(), () -> s_ClimbAvator.getShoulderMotorPosition() < Conversions.shoulderDegreesToRotations(30) || (s_ClimbAvator.getShoulderMotorPosition() > Conversions.shoulderDegreesToRotations(30) && s_ClimbAvator.getElevatorMotorPosition() - s_ClimbAvator.getState().getHeight() < 5));
  }

  public Command disperse() {
    return Commands.either(s_ClimbAvator.setShoulderNeutral().andThen(s_ClimbAvator.waitUntilShoulderNeutralSafe()), s_ClimbAvator.setShoulder().andThen(s_ClimbAvator.waitUntilShoulderSafe()), () -> s_MrPibb.turretMoving().getAsBoolean() && s_ClimbAvator.getState().getAngle() < Conversions.shoulderDegreesToRotations(30))
                           .andThen(s_MrPibb.setWristNeutral().andThen(s_MrPibb.waitUntilWristNeutralSafe()))
                           .andThen(s_ClimbAvator.setElevator()).andThen(s_MrPibb.setTurret())
                           .andThen(s_ClimbAvator.waitUntilElevatorSafe()).andThen(s_MrPibb.waitUntilTurretSafe())
                           .andThen(s_ClimbAvator.setShoulder()).andThen(s_MrPibb.setWrist())
                           .andThen(s_ClimbAvator.waitUntilShoulderSafe()).andThen(s_MrPibb.waitUntilWristSafe());
  }

  public Command condense() {
    return s_MrPibb.setWristNeutral().andThen(s_MrPibb.waitUntilWristNeutralSafe())
                   .andThen(s_ClimbAvator.setElevator()).andThen(s_MrPibb.setTurret())
                   .andThen(s_ClimbAvator.waitUntilElevatorSafe()).andThen(s_MrPibb.waitUntilTurretSafe())
                   .andThen(s_ClimbAvator.setShoulder()).andThen(s_MrPibb.setWrist())
                   .andThen(s_ClimbAvator.waitUntilShoulderSafe()).andThen(s_MrPibb.waitUntilWristSafe());
  }

  /* Intake States */
  public Command stationState() {
    return runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.STATION))
               .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.STATION)))
               .andThen(runSubsystems());
  }

  public Command preStageState() {
    return runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.CORAL_PRESTAGE))
               .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.CORAL_PRESTAGE)))
               .andThen(runSubsystems());
  }

  public Command groundCoralState() {
    return runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.GROUND_CORAL))
                .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.GROUND_CORAL)))
                .andThen(runSubsystems());
  }

  public Command groundAlgaeState() {
    return runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.GROUND_ALGAE))
                .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.GROUND_ALGAE)))
                .andThen(runSubsystems());
  }

  /* Algae States */
  public Command lowerAlgaeState() {
    return runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.LOWER_ALGAE))
               .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.LOWER_ALGAE)))
               .andThen(runSubsystems());
  }

  public Command upperAlgaeState() {
    return runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.UPPER_ALGAE))
               .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.UPPER_ALGAE)))
               .andThen(runSubsystems());
  }

  public Command bargeState() {
    return runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.BARGE))
               .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.BARGE)))
               .andThen(runSubsystems());
  }

  public Command processorState() {
    return runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.PROCESSOR))
               .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.PROCESSOR)))
               .andThen(runSubsystems());
  }

  /* Scoring States */
  public Command l1State() {
    return runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.L1))
               .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.L1)))
               .andThen(runSubsystems());
  }

  public Command l2State() {
    return runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.L2))
               .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.L2)))
               .andThen(runSubsystems());
  }

  public Command l3State() {
    return runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.L3))
               .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.L3)))
               .andThen(runSubsystems());
  }

  public Command l4State() {
    return runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.L4))
               .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.L4)))
               .andThen(runSubsystems());
  }

  /* Other States */
  public Command climbState() {
    return runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.CLIMB))
               .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.CLIMB)))
               .andThen(runSubsystems());
  }

  public Command climbPullState() {
    return runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.CLIMB_PULL))
               .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.CLIMB)))
               .andThen(runSubsystems());
  }

  public Command protectState() {
    return runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.PROTECT))
               .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.PROTECT)))
               .andThen(runSubsystems());
  }

  public BooleanSupplier botFullCoral() {
    return s_MrPibb.botFullCoral();
  }

  public BooleanSupplier botFullAlgae() {
    return s_MrPibb.botFullAlgae();
  }
}
