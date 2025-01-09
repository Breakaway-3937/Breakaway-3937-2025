// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Swerve.Swerve;

/**
 * Pass zero for tag ID to track best 
 */
public class CenterOnAprilTag extends Command {
  private Swerve swerve;
  private Vision vision;
  private int tag;
  private ProfiledPIDController alignController; //TODO Y axis???

  private final SwerveRequest.FieldCentric swerveRequest = new SwerveRequest.FieldCentric() //TODO maybe needs to be robot centric
  .withDriveRequestType(DriveRequestType.Velocity)
  .withSteerRequestType(SteerRequestType.MotionMagicExpo)
  .withVelocityX(0.0)
  .withVelocityY(0.0);

  /** Creates a new CenterOnAprilTag. */
  public CenterOnAprilTag(Swerve swerve, Vision vision, int tag) {
    this.swerve = swerve;
    this.tag = tag;
    this.vision = vision;

    alignController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(1, 1));

    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    alignController.reset(0); //TODO get info from camera
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if vision has targets 
    //check target for tag id or find best target if tag id zero passed
    //robot centric y axis. Just make it center

    //if no targets end command??????
    //Maybe allows default comamnd to take over with its rotation controls from the joysick????
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
