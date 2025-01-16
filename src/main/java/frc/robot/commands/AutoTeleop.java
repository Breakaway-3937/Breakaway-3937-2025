// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.OperatorController;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Swerve.Swerve;

public class AutoTeleop extends SequentialCommandGroup {
  private Swerve s_Swerve;
  private Vision s_Vision;

  /** Creates a new AutoTelop. */
  public AutoTeleop(Swerve s_Swerve, Vision s_Vision) {
    this.s_Swerve = s_Swerve;
    this.s_Vision = s_Vision;

    addCommands(s_Swerve.pathFindToPose(OperatorController.getPickUpLocation()), 
                new WaitCommand(5),
                s_Swerve.pathFindToPose(OperatorController.getScoringLocation()));
  }
}