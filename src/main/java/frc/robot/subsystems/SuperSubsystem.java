// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.Conversions;
import frc.robot.subsystems.ClimbAvator.ClimbAvator;
import frc.robot.subsystems.ClimbAvator.ClimbAvatorStates;
import frc.robot.subsystems.LED.LEDStates;
import frc.robot.subsystems.Soda.DrPepper;
import frc.robot.subsystems.Soda.MrPibb;
import frc.robot.subsystems.Soda.MrPibbStates;

public class SuperSubsystem extends SubsystemBase {
  private final ClimbAvator s_ClimbAvator;
  private final MrPibb s_MrPibb;
  private final DrPepper s_DrPepper;
  private final LED s_LED;
  private final BooleanSupplier /*funeral,*/ isBackwards;
  //Temp
  boolean funeral = false;

  /** Creates a new SuperSubsystem. */
  public SuperSubsystem(ClimbAvator s_ClimbAvator, MrPibb s_MrPibb, DrPepper s_DrPepper, LED s_LED, /*BooleanSupplier funeral*/ boolean funeral, BooleanSupplier isBackwards) {
    this.s_ClimbAvator = s_ClimbAvator; 
    this.s_MrPibb = s_MrPibb;
    this.s_DrPepper = s_DrPepper;
    this.s_LED = s_LED;
    this.funeral = funeral;
    this.isBackwards = isBackwards;
  }

  public Command saveMrPibb() {
    return Commands.either(s_ClimbAvator.configShoulderFast().andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.PROTECT))).andThen(s_MrPibb.setWrist()).andThen(s_MrPibb.waitUntilWristSafe()), Commands.none(), () -> s_ClimbAvator.getState().equals(ClimbAvatorStates.CLIMB_PULL))
                    .andThen(Commands.either(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.PROTECT)).andThen(s_MrPibb.setWrist()).andThen(s_MrPibb.waitUntilWristSafe()), Commands.none(), () -> s_ClimbAvator.getState().equals(ClimbAvatorStates.GROUND_ALGAE)));
  }

  public Command runSubsystems() {
    return Commands.either(disperse(), condense(), () -> s_ClimbAvator.getShoulderMotorPosition() < Conversions.shoulderDegreesToRotations(30) || s_ClimbAvator.getElevatorMotorPosition() - s_ClimbAvator.getState().getHeight() < 5);
  }

  public Command disperse() {
    return Commands.either(s_MrPibb.setTurretZero(), Commands.none(), () -> s_ClimbAvator.getState().getAngle() < Conversions.shoulderDegreesToRotations(30) && Math.abs(s_MrPibb.getTurretPosition() - 0) > 0.35).andThen(s_ClimbAvator.setShoulder()).andThen(s_ClimbAvator.waitUntilShoulderSafe())
                           .andThen(s_MrPibb.setWrist()).andThen(s_ClimbAvator.setElevator()).andThen(s_MrPibb.setTurret())
                           .andThen(s_MrPibb.waitUntilWristSafe()).andThen(s_ClimbAvator.waitUntilElevatorSafe()).andThen(s_MrPibb.waitUntilTurretSafe());
  }

  public Command condense() {
    return s_MrPibb.setWrist().andThen(s_ClimbAvator.setElevator()).andThen(s_MrPibb.setTurret())
                   .andThen(s_MrPibb.waitUntilWristSafe()).andThen(s_ClimbAvator.waitUntilElevatorSafe()).andThen(s_MrPibb.waitUntilTurretSafe())
                   .andThen(s_ClimbAvator.setShoulder()).andThen(s_ClimbAvator.waitUntilShoulderSafe());
  }

  /* Intake States */
  public Command stationState() {
    return saveMrPibb().andThen(runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.STATION)))
               .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.STATION)))
               .andThen(runSubsystems());
  }

  public Command preStageState() {
    return saveMrPibb().andThen(runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.CORAL_PRESTAGE)))
               .andThen(Commands.either(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.CORAL_PRESTAGE)), runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.EMPTY_PRESTATE)), s_DrPepper.botFullCoral()))
               .andThen(runSubsystems());
  }

  public Command groundCoralState() {
    return saveMrPibb().andThen(runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.GROUND_CORAL)))
                .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.GROUND_CORAL)))
                .andThen(runSubsystems());
  }

  public Command lollipopState() {
    return saveMrPibb().andThen(runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.LOLLIPOP)))
                .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.LOLLIPOP)))
                .andThen(runSubsystems());
  }

  public Command groundAlgaeState() {
    return saveMrPibb().andThen(runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.GROUND_ALGAE)))
                .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.GROUND_ALGAE)))
                .andThen(runSubsystems());
  }

  /* Algae States */
  public Command lowerAlgaeState() {
    return saveMrPibb().andThen(runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.LOWER_ALGAE)))
               .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.LOWER_ALGAE)))
               .andThen(runSubsystems());
  }

  public Command upperAlgaeState() {
    return saveMrPibb().andThen(runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.UPPER_ALGAE)))
               .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.UPPER_ALGAE)))
               .andThen(runSubsystems());
  }

  public Command bargeState() {
    return saveMrPibb().andThen(runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.BARGE))
                .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.BARGE))))
                .andThen(runSubsystems());
  }

  public Command processorState() {
    return saveMrPibb().andThen(runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.PROCESSOR)))
               .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.PROCESSOR)))
               .andThen(s_MrPibb.setWrist())
               .andThen(runSubsystems());
  }

  /* Scoring States */
  public Command l1State() {
    return saveMrPibb().andThen(runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.L1)))
               .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.L1)))
               .andThen(runSubsystems());
  }

  public Command l2State() {
    return saveMrPibb().andThen(
           Commands.either(
                    runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.BACKWARDS_L2))
                    .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.BACKWARDS_L2))), 
                    runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.L2))
                    .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.L2))), isBackwards))
               .andThen(runSubsystems());
  }

  public Command l3State() {
    return saveMrPibb().andThen(
           Commands.either(
                    runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.BACKWARDS_L3))
                    .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.BACKWARDS_L3))), 
                    runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.L3))
                    .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.L3))), isBackwards))
                .andThen(Commands.either(
                      s_ClimbAvator.setShoulder().andThen(s_ClimbAvator.waitUntilShoulderSafe())
                      .andThen(s_ClimbAvator.setElevator()).andThen(s_MrPibb.setTurret())
                      .andThen(s_MrPibb.waitUntilTurretSafe()).andThen(s_ClimbAvator.waitUntilElevatorSafe())
                      .andThen(s_MrPibb.setWrist()).andThen(s_MrPibb.waitUntilWristSafe()),
                      runSubsystems(), isBackwards));
  }

  public Command l4State() {
    return saveMrPibb().andThen(
           Commands.either(
                    runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.BACKWARDS_L4))
                    .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.BACKWARDS_L4))), 
                    runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.L4))
                    .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.L4))), isBackwards))
               .andThen(Commands.either(
                    s_ClimbAvator.setShoulder().andThen(s_ClimbAvator.waitUntilShoulderSafe())
                    .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.POOPY_BACKWARDS_L4)))
                    .andThen(s_MrPibb.setWrist()).andThen(s_MrPibb.waitUntilWristSafe())
                    .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.BACKWARDS_L4)))
                    .andThen(s_ClimbAvator.setElevator()).andThen(s_MrPibb.setTurret())
                    .andThen(s_MrPibb.waitUntilTurretSafe()).andThen(s_ClimbAvator.waitUntilElevatorSafe())
                    .andThen(s_MrPibb.setWrist()).andThen(s_MrPibb.waitUntilWristSafe()),
                    runSubsystems(), isBackwards));
  }

  /* Other States */
  public Command climbState() {
    return Commands.either(runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.CLIMB))
               .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.BARGE)))
               .andThen(runSubsystems())
               .andThen(s_ClimbAvator.preStageBilboBagginsTheBack()),
               Commands.none(),
               () -> s_ClimbAvator.getState().equals(ClimbAvatorStates.CORAL_PRESTAGE));
  }

  public Command climbPullState() {
    return Commands.either(s_ClimbAvator.configShoulderSlow().andThen(runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.CLIMB_PULL)))
               .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.CLIMB)))
               .andThen(s_ClimbAvator.setShoulder()).andThen(s_ClimbAvator.waitUntilShoulderSafe())
               .andThen(s_ClimbAvator.setElevator()).andThen(s_ClimbAvator.waitUntilElevatorSafe())
               .andThen(s_MrPibb.setTurret()).andThen(s_MrPibb.waitUntilTurretSafe())
               .andThen(s_MrPibb.setWrist()).andThen(s_MrPibb.waitUntilWristSafe()),
               Commands.none(),
               () -> s_ClimbAvator.getState().equals(ClimbAvatorStates.CLIMB))
               .andThen(Commands.runOnce(() -> s_LED.setState(LEDStates.CLIMBED)));
  }

  public Command protectState() {
    return saveMrPibb().andThen(runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.PROTECT)))
               .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.PROTECT)))
               .andThen(runSubsystems());
  }

  public BooleanSupplier botFullCoral() {
    return s_DrPepper.botFullCoral();
  }

  public BooleanSupplier botFullAlgae() {
    return s_DrPepper.botFullAlgae();
  }

  public BooleanSupplier isAlgae() {
    return () -> getClimbAvatorState().equals(ClimbAvatorStates.GROUND_ALGAE) || getClimbAvatorState().equals(ClimbAvatorStates.LOWER_ALGAE) || getClimbAvatorState().equals(ClimbAvatorStates.UPPER_ALGAE) || getClimbAvatorState().equals(ClimbAvatorStates.PROCESSOR) || getClimbAvatorState().equals(ClimbAvatorStates.BARGE);
  }

  public Command hitReef(Command hit, Command stop) {
    return new ParallelDeadlineGroup(Commands.waitSeconds(0.45), hit).andThen(Commands.waitSeconds(0.01).raceWith(stop)).andThen(() -> stop.cancel());
  }

  public Command pickup(Command hit, Command stop) {
    return hitReef(hit, stop).andThen(lowerAlgaeState()).andThen(s_DrPepper.runLoader()).andThen(Commands.waitUntil(botFullAlgae()));
  }

  public Command scoreAlgae() {
    return s_DrPepper.runLoaderReverse().andThen(Commands.waitSeconds(0.5).andThen(s_DrPepper.stopLoader()));
  }

  public Command scoreCoral(Command hit, Command stop) {
    return hitReef(hit, stop).andThen(l4State()).andThen(s_DrPepper.runThumbForward()).andThen(s_DrPepper.runLoaderSlowly()).andThen(Commands.waitSeconds(0.5).andThen(s_DrPepper.stopThumb()).andThen(s_DrPepper.stopLoader()));
  }

  public Command scoreCoralAlign(Command hit) {
    return hit.andThen(l4State()).andThen(s_DrPepper.runThumbForward()).andThen(s_DrPepper.runLoaderSlowly()).andThen(Commands.waitSeconds(0.5).andThen(s_DrPepper.stopThumb()).andThen(s_DrPepper.stopLoader()));
  }

  public Command scoreCoralAlignFast(Command hit) {
    return hit.andThen(s_ClimbAvator.waitUntilElevatorSafe()).andThen(s_DrPepper.runThumbForward()).andThen(s_DrPepper.runLoaderSlowly()).andThen(Commands.waitSeconds(0.5).andThen(s_DrPepper.stopThumb()).andThen(s_DrPepper.stopLoader()));
  }
  
  public Command scoreCoralL1(Command hit, Command stop) {
    return hitReef(hit, stop).andThen(l1State()).andThen(s_DrPepper.runLoaderReverseTrough()).andThen(Commands.waitSeconds(0.5).andThen(s_DrPepper.stopLoader()));
  }

  public Command load() {
    return (s_DrPepper.runLoader().andThen(Commands.waitUntil(botFullCoral())));
  }

  public Command loadBriefly() {
    return Commands.waitSeconds(0.5).andThen(s_DrPepper.stopLoader());
  }

  public Command center() {
    return s_DrPepper.autoCenter().andThen(s_DrPepper.stopThumb());
  }

  public Command tushPush(Command hit, Command stop) {
    return new ParallelDeadlineGroup(Commands.waitSeconds(0.75), hit).andThen(Commands.waitSeconds(0.01).raceWith(stop)).andThen(() -> stop.cancel());
  }

  public Command condenseAuto() {
    return saveMrPibb().andThen(runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.CORAL_PRESTAGE)))
                       .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.CORAL_PRESTAGE)))
                       .andThen(s_MrPibb.setWrist()).andThen(s_MrPibb.setTurret()).andThen(s_ClimbAvator.setElevator())
                       .andThen(s_MrPibb.waitUntilWristSafe()).andThen(s_MrPibb.waitUntilTurretSafe()).andThen(s_ClimbAvator.waitUntilElevatorSafe())
                       .andThen(runOnce(() -> s_ClimbAvator.setClimbAvatorState(ClimbAvatorStates.STATION)))
                       .andThen(runOnce(() -> s_MrPibb.setMrPibbState(MrPibbStates.STATION)))
                       .andThen(s_MrPibb.setWrist()).andThen(s_MrPibb.setTurret()).andThen(s_ClimbAvator.setElevator())
                       .andThen(s_MrPibb.waitUntilWristSafe()).andThen(s_MrPibb.waitUntilTurretSafe()).andThen(s_ClimbAvator.waitUntilElevatorSafe())
                       .andThen(s_ClimbAvator.setShoulder()).andThen(s_ClimbAvator.waitUntilShoulderSafe());
  }

  public ClimbAvatorStates getClimbAvatorState() {
    return s_ClimbAvator.getState();
  }

  public MrPibbStates getMrPibbState() {
    return s_MrPibb.getStateAsEnum();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    var command = getCurrentCommand();
    if(command != null) {
      Logger.recordOutput("SuperSubsystem/Current Command", command.getName());
    }

    if(DriverStation.isEnabled() && /*!funeral.getAsBoolean()*/ !funeral && !s_ClimbAvator.getState().equals(ClimbAvatorStates.CLIMB_PULL) && !s_LED.getState().equals(LEDStates.BOT_ALIGNING)) {
      if(botFullAlgae().getAsBoolean() && !s_LED.getState().equals(LEDStates.ALGAE_FULL)) {
        s_LED.setState(LEDStates.ALGAE_FULL);
      }
      else if(s_DrPepper.botFullCoral().getAsBoolean() && !s_LED.getState().equals(LEDStates.CORAL_FULL)) {
        s_LED.setState(LEDStates.CORAL_FULL);
      }
      else if(!botFullAlgae().getAsBoolean() && !s_DrPepper.botFullCoral().getAsBoolean()) {
        s_LED.setState(LEDStates.BOT_EMPTY);
      }
    }
  }
}
