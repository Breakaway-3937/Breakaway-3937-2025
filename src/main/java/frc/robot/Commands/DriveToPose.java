// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class DriveToPose extends Command {
  private Swerve swerve;
  private ProfiledPIDController xController, yController;
  private Pose2d targetPose, robotPose;
  private double xSpeed, ySpeed;


  private final SwerveRequest.FieldCentricFacingAngle swerveRequestFacing = new SwerveRequest.FieldCentricFacingAngle()
  .withDriveRequestType(DriveRequestType.Velocity)
  .withSteerRequestType(SteerRequestType.MotionMagicExpo)
  .withVelocityX(0.0)
  .withVelocityY(0.0);

  /** Creates a new DriveToPose. */
  public DriveToPose(Swerve swerve, Pose2d targetPose) {
    xController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(1, .5));
    yController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(1, .5));

    swerveRequestFacing.HeadingController = new PhoenixPIDController(2, 0, 0);
    swerveRequestFacing.HeadingController.setTolerance(0.1);
    swerveRequestFacing.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    
    this.swerve = swerve;
    this.targetPose = targetPose;

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    robotPose = swerve.getState().Pose;

    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
    swerveRequestFacing.HeadingController.reset();

    xController.setGoal(targetPose.getX());
    yController.setGoal(targetPose.getY());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    robotPose = swerve.getState().Pose;

    xController.setGoal(targetPose.getX());
    yController.setGoal(targetPose.getY());

    xSpeed = (!xController.atGoal()) ? xController.calculate(robotPose.getX()) : 0;
    ySpeed = (!yController.atGoal()) ? yController.calculate(robotPose.getY()) : 0;
    //System.out.println("X: " + xSpeed + " Y: " + ySpeed);

    swerve.setControl(swerveRequestFacing.withVelocityX(-xSpeed)
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
