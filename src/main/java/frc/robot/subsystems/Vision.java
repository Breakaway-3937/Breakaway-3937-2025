// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.Vision.BreakaCamera;
import frc.robot.subsystems.Swerve.Swerve;

public class Vision extends SubsystemBase {
  private AprilTagFieldLayout atfl;
  private BreakaCamera camera;
  private Swerve swerve;

  /** Creates a new Vision. */
  public Vision(Swerve swerve) {
    this.swerve = swerve;
    try {
      atfl = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
    } catch(IOException e) {
      e.printStackTrace();
    }

    camera = new BreakaCamera("FrontCamera", new PhotonPoseEstimator(atfl, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.Vision.FRONT_CAMERA_TRANSFORM));    
  }

  @Override
  public void periodic() {
    var result = camera.getEstimatedPose();
    if(!result.isEmpty()) {
      System.out.println("UPDATE");
      swerve.addVisionMeasurement(result.get().estimatedPose.toPose2d(), result.get().timestampSeconds, Constants.Vision.TAG_VISION_STDS_FRONT);
    }
  }

}