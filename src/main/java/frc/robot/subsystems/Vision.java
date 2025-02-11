// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import frc.robot.lib.Vision.BreakaCamera;
import frc.robot.subsystems.Swerve.Swerve;

public class Vision extends SubsystemBase {
  private AprilTagFieldLayout atfl;
  private final BreakaCamera camera;
  private final BreakaCamera otherCamera;
  private final Swerve s_Swerve;
  private final double maxDistance = 6;
  private boolean frontCameraBad;
  private ArrayList<Pose3d> frontTagsUsed;

  /** Creates a new Vision. */
  public Vision(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
    
    try {
      atfl = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
    } catch(IOException e) {
      e.printStackTrace();
    }

    camera = new BreakaCamera("FrontCamera", new PhotonPoseEstimator(atfl, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.Vision.FRONT_CAMERA_TRANSFORM));
    otherCamera = new BreakaCamera("3937", new PhotonPoseEstimator(atfl, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.Vision.BACK_CAMERA_TRANSFORM));
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

  public Vector<N3> std(double averageDistance) {
    return VecBuilder.fill(0, 0, 0); 
    // Make an Interpolating map that uses average distance from camera to find x and y std. 
    // Fill map with varing ranges and stds from aScope
  }

  @Override
  public void periodic() {
    var result = camera.getEstimatedPose();
    var otherResult = otherCamera.getEstimatedPose();

    if(!result.isEmpty()) {
      double averageDistance = 0;

      for(int i = 0; i < result.get().targetsUsed.size(); i++) {
        averageDistance += Math.abs(result.get().targetsUsed.get(i).getBestCameraToTarget().getX());
      }
      averageDistance /= (double) result.get().targetsUsed.size();

      if(averageDistance > maxDistance) {
        frontCameraBad = true;
      }

      if(!frontCameraBad) {
        s_Swerve.addVisionMeasurement(result.get().estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(result.get().timestampSeconds), Constants.Vision.TAG_VISION_STDS_FRONT);
      }
    }
    if(!otherResult.isEmpty()) {
      s_Swerve.addVisionMeasurement(otherResult.get().estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(otherResult.get().timestampSeconds), Constants.Vision.TAG_VISION_STDS_FRONT);
    }

    if(!result.isEmpty()) {
      for(int i = 0; i < result.get().targetsUsed.size(); i++) {
        frontTagsUsed.add(camera.getPhotonPoseEstimator().getFieldTags().getTagPose(result.get().targetsUsed.get(i).getFiducialId()).get());
      }

      if(!frontTagsUsed.isEmpty()) {
        Logger.recordOutput("Front Camera Tags Used", frontTagsUsed.toArray(new Pose3d[frontTagsUsed.size()]));
      }
    }

    Logger.recordOutput("Front Camera Connected", camera.isDead());
  }
}
