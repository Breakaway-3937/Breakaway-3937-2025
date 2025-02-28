// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Swerve.AutoPathLocations;
import frc.robot.subsystems.Swerve.Swerve;

/**
 * Pass zero for tag ID to track best target.
 */
public class HoldPosition extends Command {
  private final Swerve s_Swerve;
  private final Vision s_Vision;
  private final ProfiledPIDController controller;
  private final Joystick joystick;
  private PhotonPipelineResult result;

  private final SwerveRequest.FieldCentricFacingAngle swerveRequest = new SwerveRequest.FieldCentricFacingAngle()
    .withDriveRequestType(DriveRequestType.Velocity)
    .withSteerRequestType(SteerRequestType.MotionMagicExpo)
    .withVelocityX(0.0)
    .withVelocityY(0.0);

  /** Creates a new CenterOnAprilTag. */
  public HoldPosition(Swerve s_Swerve, Vision s_Vision, Joystick joystick) {
    this.s_Swerve = s_Swerve;
    this.s_Vision = s_Vision;
    this.joystick = joystick;

    //TODO: Get the correct PID values.
    controller = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(1, 1));

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Swerve, s_Vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    result = s_Vision.getLatestFront();
    if(result.hasTargets()) {
      controller.reset(result.getBestTarget().getBestCameraToTarget().getY());
    }
    controller.setGoal(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(result.hasTargets()) {
      var ySpeed = ChassisSpeeds.fromRobotRelativeSpeeds(0.0, controller.calculate(result.getBestTarget().getBestCameraToTarget().getY()), 0.0, s_Swerve.getState().Pose.getRotation()).vyMetersPerSecond;
      s_Swerve.setControl(swerveRequest.withVelocityX(joystick.getX())
                                       .withVelocityY(ySpeed)
                                       .withTargetDirection(AutoPathLocations.valueOf("A").getRotationTarget()));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //TODO: Add the correct tolerance.
    return false;
  }
}