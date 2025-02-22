// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;

import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.OperatorController;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Swerve.Swerve;


public class CenterOnAprilTag extends Command {
  private final Swerve s_Swerve;
  private final Vision s_Vision;
  private final Joystick translation;
  private final ProfiledPIDController driveController, turnController;
  private int tag = 0;
  private PhotonPipelineResult result;

  private final SwerveRequest.RobotCentric swerveRequest = new SwerveRequest.RobotCentric()
    .withDriveRequestType(DriveRequestType.Velocity)
    .withSteerRequestType(SteerRequestType.MotionMagicExpo)
    .withVelocityX(0.0)
    .withVelocityY(0.0)
    .withRotationalRate(0.0);

  /** Creates a new CenterOnAprilTag. */
  public CenterOnAprilTag(Swerve s_Swerve, Vision s_Vision, Joystick translation) {
    this.s_Swerve = s_Swerve;
    this.s_Vision = s_Vision;
    this.translation = translation;

    driveController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(1, 1));
    turnController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(Math.PI / 2.0, Math.PI / 2.0));

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Swerve, s_Vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    result = s_Vision.getLatestFront();
    if(result.hasTargets()) {
      tag = result.getBestTarget().getFiducialId();
      if(goodTag(tag)) {
        driveController.reset(result.getBestTarget().getBestCameraToTarget().getY());
        turnController.reset(getGyro());

        turnController.setGoal(getTargetAngle(tag));

        if(scoringLocationLeft()) {
          driveController.setGoal(-0.14);
        }
        else {
          driveController.setGoal(0.14);
        }
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(goodTag(tag)) {
      System.out.println(getTargetAngle(tag));
      result = s_Vision.getLatestFront();
      if(result.hasTargets()) {
        s_Swerve.setControl(swerveRequest.withVelocityX(translation.getRawAxis(Constants.Controllers.TRANSLATION_AXIS) * Constants.Swerve.MAX_SPEED)
                                        .withVelocityY(-driveController.calculate(result.getBestTarget().getBestCameraToTarget().getY()) * Constants.Swerve.MAX_SPEED)
                                        .withRotationalRate(turnController.calculate(getGyro())));
      }
    }
    else {
      s_Swerve.setControl(swerveRequest);
    }
  }

  public double getGyro() {
    return Units.Radians.convertFrom(s_Swerve.getPigeon2().getYaw().getValueAsDouble(), Units.Degrees) + 600.0 * Math.PI % 2.0 * Math.PI;
  }

  public double getTargetAngle(int tag) {
    switch(tag) {
      case 6:
        return 2.0 / 3.0 * Math.PI;
      case 7:
        return Math.PI;
      case 8:
        return 4.0 / 3.0 * Math.PI;
      case 9:
        return 5.0 / 3.0 * Math.PI;
      case 10: 
        return 0.0; 
      case 11:
        return 1.0 / 3.0 * Math.PI;      
      case 17:
        return 1.0 / 3.0 * Math.PI;
      case 18: 
        return 0.0; 
      case 19:
        return 5.0 / 3.0 * Math.PI;
      case 20: 
        return 4.0 / 3.0 * Math.PI;
      case 21:
        return Math.PI;
      case 22: 
        return 2.0 / 3.0 * Math.PI;
    }
    return 0.0;
  }

  public boolean goodTag(int tag) {
    return getTargetAngle(tag) !=0 || tag == 10 || tag == 18;
  }

  public boolean scoringLocationLeft() {
    String location = OperatorController.getScoringLocation().get().name();
    if(location.equals("A") || location.equals("C") || location.equals("E")
      || location.equals("G") || location.equals("I") || location.equals("K")) {
        return true;
    }
    else {
      return false;
    }
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
