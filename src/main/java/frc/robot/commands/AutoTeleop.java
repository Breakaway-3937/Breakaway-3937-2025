// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;
import java.util.function.Supplier;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Swerve.AutoPathLocations;
import frc.robot.subsystems.Swerve.Swerve;

public class AutoTeleop extends SequentialCommandGroup {
  private Swerve s_Swerve;
  private Vision s_Vision;
  private Supplier<AutoPathLocations> pickup, score;

  /** Creates a new AutoTelop. */
  public AutoTeleop(Swerve s_Swerve, Vision s_Vision, Supplier<AutoPathLocations> pickup, Supplier<AutoPathLocations> score) {
    this.s_Swerve = s_Swerve;
    this.s_Vision = s_Vision;
    this.pickup = pickup;
    this.score = score;

    addRequirements(s_Swerve, s_Vision);

    addCommands(pathFindPickup(), pathFindScore());
  }

  private Command pathFindPickup() {
    return Commands.defer(
      () -> {
        return s_Swerve.pathFindToPose(pickup);
      },
      Set.of(s_Swerve)
    );
  }

  private Command pathFindScore() {
    return Commands.defer(
      () -> {
        return s_Swerve.pathFindToPose(score);
      },
      Set.of(s_Swerve)
    );
  }
}