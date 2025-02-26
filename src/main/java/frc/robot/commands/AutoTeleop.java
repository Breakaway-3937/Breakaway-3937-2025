// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.OperatorController;
import frc.robot.subsystems.SuperSubsystem;
import frc.robot.subsystems.Swerve.Swerve;

public class AutoTeleop extends SequentialCommandGroup {
  private final Swerve s_Swerve;
  private final SuperSubsystem s_SuperSubsystem;

  /** Creates a new AutoTeleop. */
  public AutoTeleop(Swerve s_Swerve, SuperSubsystem s_SuperSubsystem) {
    this.s_Swerve = s_Swerve;
    this.s_SuperSubsystem = s_SuperSubsystem;

    setName("AutoTeleop");
    addRequirements(s_Swerve);

    addCommands(pathFindPickup(), 
                pathFindScore(),
                OperatorController.clearLocations());
  }

  private Command pathFindPickup() {
    return Commands.defer(
      () -> {
        return s_Swerve.pathFindAndFollow(OperatorController.getPickUpLocation()).unless(s_SuperSubsystem.botFullCoral());
      },
      Set.of(s_Swerve)
    );
  }

  private Command pathFindScore() {
    return Commands.defer(
      () -> {
        return s_Swerve.pathFindAndFollow(OperatorController.getScoringLocation());
      },
      Set.of(s_Swerve)
    );
  }

  private BooleanSupplier robotAtPickUp() {
    var location = OperatorController.getPickUpLocation();
    if(location.get() != null && location.get().getLocation() != null) {
      boolean nearX = MathUtil.isNear(location.get().getLocation().getX(), s_Swerve.getState().Pose.getX(), 0);
      boolean nearY = MathUtil.isNear(location.get().getLocation().getY(), s_Swerve.getState().Pose.getY(), 0);
      return () -> nearX && nearY;
    }
    else {
      return () -> false;
    }
  }

  private BooleanSupplier robotAtScoring() {
    var location = OperatorController.getScoringLocation();
    if(location.get() != null && location.get().getLocation() != null) {
      boolean nearX = MathUtil.isNear(location.get().getLocation().getX(), s_Swerve.getState().Pose.getX(), 0);
      boolean nearY = MathUtil.isNear(location.get().getLocation().getY(), s_Swerve.getState().Pose.getY(), 0);
      return () -> nearX && nearY;
    }
    else {
      return () -> false;
    }
  }
}