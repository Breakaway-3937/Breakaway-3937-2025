// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.Vision.BreakaCamera;
import frc.robot.subsystems.Swerve.Swerve;

public class Vision extends SubsystemBase {
  private AprilTagFieldLayout atfl;
  private final BreakaCamera camera;
  private final Swerve s_Swerve;

  /** Creates a new Vision. */
  public Vision(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
    
    try {
      atfl = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
    } catch(IOException e) {
      e.printStackTrace();
    }

    camera = new BreakaCamera("FrontCamera", new PhotonPoseEstimator(atfl, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.Vision.FRONT_CAMERA_TRANSFORM));
  }

  public double getX() {
    return camera.getLatest().hasTargets() ? camera.getLatest().getBestTarget().getBestCameraToTarget().getX() : 0;
  }

  public boolean hasTargets() {
    return camera.getLatest().hasTargets();
  }

  public PhotonPipelineResult getLatest() {
    return camera.getLatest();
  }

  @Override
  public void periodic() {
    var result = camera.getEstimatedPose();
    if(!result.isEmpty()) {
      double x = result.get().estimatedPose.toPose2d().getX();
      double y = result.get().estimatedPose.toPose2d().getY();
      Pose2d pose = new Pose2d(x, y, s_Swerve.getState().Pose.getRotation());
      
      s_Swerve.addVisionMeasurement(pose, Utils.fpgaToCurrentTime(result.get().timestampSeconds), Constants.Vision.TAG_VISION_STDS_FRONT);
    }
  }

}