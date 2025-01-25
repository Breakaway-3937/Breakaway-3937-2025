// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.OperatorController;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Swerve.Swerve;

public class AutoTeleop extends SequentialCommandGroup {
  private final Swerve s_Swerve;
  private final Vision s_Vision;

  //TODO: Add the aligning command.
  /** Creates a new AutoTelop. */
  public AutoTeleop(Swerve s_Swerve, Vision s_Vision) {
    this.s_Swerve = s_Swerve;
    this.s_Vision = s_Vision;

    addRequirements(s_Swerve, s_Vision);

    //Change the until at pose
    addCommands(pathFindPickup().until(() -> false), pathFindScore());
  }

  private Command pathFindPickup() {
    return Commands.defer(
      () -> {
        //change the unless to bot full
        return s_Swerve.pathFindToPose(OperatorController.getPickUpLocation()).unless(() -> false);
      },
      Set.of(s_Swerve)
    );
  }

  private Command pathFindScore() {
    return Commands.defer(
      () -> {
        return s_Swerve.pathFindToPose(OperatorController.getScoringLocation());
      },
      Set.of(s_Swerve)
    );
  }
}