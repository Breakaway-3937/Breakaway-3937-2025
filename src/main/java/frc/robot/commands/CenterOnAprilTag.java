// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Swerve.Swerve;

/**
 * Pass zero for tag ID to track best target.
 */
public class CenterOnAprilTag extends Command {
  private final Swerve s_Swerve;
  private final Vision s_Vision;
  private final int tag;
  private final ProfiledPIDController controller;

  private final SwerveRequest.RobotCentric swerveRequest = new SwerveRequest.RobotCentric()
    .withDriveRequestType(DriveRequestType.Velocity)
    .withSteerRequestType(SteerRequestType.MotionMagicExpo)
    .withVelocityX(0.0)
    .withVelocityY(0.0);

  /** Creates a new CenterOnAprilTag. */
  public CenterOnAprilTag(Swerve s_Swerve, Vision s_Vision, int tag) {
    this.s_Swerve = s_Swerve;
    this.tag = tag;
    this.s_Vision = s_Vision;

    //TODO: Get the correct PID values.
    controller = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(1, 1));

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Swerve, s_Vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var result = s_Vision.getLatest();
    if(result.hasTargets()) {
      controller.reset(result.getBestTarget().getBestCameraToTarget().getY());
    }
    controller.setGoal(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var result = s_Vision.getLatest();
    if(result.hasTargets()) {
      if(tag == 0) {
        s_Swerve.setControl(swerveRequest.withVelocityY(controller.calculate(result.getBestTarget().getBestCameraToTarget().getY())));
      }
      else {
        for(PhotonTrackedTarget target : result.getTargets()) {
          if(target.getFiducialId() == tag) {
            s_Swerve.setControl(swerveRequest.withVelocityY(controller.calculate(target.getBestCameraToTarget().getY())));
          }
        }
      }
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
