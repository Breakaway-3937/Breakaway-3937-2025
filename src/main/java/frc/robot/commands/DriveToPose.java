// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.Swerve;

public class DriveToPose extends Command {
  private final Swerve s_Swerve;
  private final ProfiledPIDController xController, yController;
  private Pose2d targetPose, robotPose;
  private double xSpeed, ySpeed;

  private final SwerveRequest.FieldCentricFacingAngle swerveRequestFacing = new SwerveRequest.FieldCentricFacingAngle()
    .withDriveRequestType(DriveRequestType.Velocity)
    .withSteerRequestType(SteerRequestType.MotionMagicExpo)
    .withVelocityX(0.0)
    .withVelocityY(0.0);

  /** Creates a new DriveToPose. */
  public DriveToPose(Swerve s_Swerve, Pose2d targetPose) {
    //FIXME: Get the correct PID values.
    xController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(1, .5));
    yController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(1, .5));

    swerveRequestFacing.HeadingController = new PhoenixPIDController(2, 0, 0);
    swerveRequestFacing.HeadingController.setTolerance(0.1);
    swerveRequestFacing.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    
    this.s_Swerve = s_Swerve;
    this.targetPose = targetPose;

    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    robotPose = s_Swerve.getState().Pose;

    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
    swerveRequestFacing.HeadingController.reset();

    xController.setGoal(targetPose.getX());
    yController.setGoal(targetPose.getY());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    robotPose = s_Swerve.getState().Pose;

    xController.setGoal(targetPose.getX());
    yController.setGoal(targetPose.getY());

    xSpeed = (!xController.atGoal()) ? xController.calculate(robotPose.getX()) : 0;
    ySpeed = (!yController.atGoal()) ? yController.calculate(robotPose.getY()) : 0;

    s_Swerve.setControl(swerveRequestFacing.withVelocityX(-xSpeed)
                                         .withVelocityY(-ySpeed) 
                                         .withTargetDirection(targetPose.getRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
