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
import frc.robot.subsystems.Swerve.Swerve;

public class AutoTeleop extends SequentialCommandGroup {
  private final Swerve s_Swerve;

  /** Creates a new AutoTeleop. */
  public AutoTeleop(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;

    setName("AutoTeleop");
    addRequirements(s_Swerve);

    addCommands(pathFindScore());
  }

  private Command pathFindScore() {
    return Commands.defer(
      () -> {
        return s_Swerve.pathFindAndFollow(OperatorController.getScoringLocation());
      },
      Set.of(s_Swerve)
    );
  }

  @SuppressWarnings("unused")
  private BooleanSupplier robotAtScoring() {
    var location = OperatorController.getScoringLocation();
    if(location.get() != null && location.get().getPath() != null) {
      var points = location.get().getPath().getAllPathPoints();
      boolean nearX = MathUtil.isNear(points.get(points.size() - 1).position.getX(), s_Swerve.getState().Pose.getX(), 0);
      boolean nearY = MathUtil.isNear(points.get(points.size() - 1).position.getY(), s_Swerve.getState().Pose.getY(), 0);
      return () -> nearX && nearY;
    }
    else {
      return () -> false;
    }
  }
}