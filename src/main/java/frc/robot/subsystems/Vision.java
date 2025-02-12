// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
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
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import frc.robot.lib.Vision.BreakaCamera;
import frc.robot.subsystems.Swerve.Swerve;

public class Vision extends SubsystemBase {
  private AprilTagFieldLayout atfl;
  private final BreakaCamera frontCamera;
  private final BreakaCamera backCamera;
  private final Swerve s_Swerve;
  private final double maxDistance = 6; // In meters
  private final InterpolatingDoubleTreeMap xStdMap, yStdMap; //Key = distance, Value = STD
  private boolean frontCameraBad, backCameraBad;
  private ArrayList<Pose3d> frontTagsUsed, backTagsUsed;

  /** Creates a new Vision. */
  public Vision(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
    
    try {
      atfl = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
    } catch(IOException e) {
      e.printStackTrace();
    }

    frontCamera = new BreakaCamera(Constants.Vision.FRONT_CAMERA_NAME, new PhotonPoseEstimator(atfl, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.Vision.FRONT_CAMERA_TRANSFORM));
    backCamera = new BreakaCamera(Constants.Vision.BACK_CAMERA_NAME, new PhotonPoseEstimator(atfl, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.Vision.BACK_CAMERA_TRANSFORM));
  
    xStdMap = new InterpolatingDoubleTreeMap();
    yStdMap = new InterpolatingDoubleTreeMap();

    frontTagsUsed = new ArrayList<Pose3d>();
    backTagsUsed = new ArrayList<Pose3d>();
  }

  public boolean hasTargets() {
    return frontCamera.getLatest().hasTargets();
  }

  public PhotonPipelineResult getLatestFront() {
    return frontCamera.getLatest();
  }

  public double getAverageTagDistanceX(Optional<EstimatedRobotPose> result) {
    double averageDistance = 0;
    for(int i = 0; i < result.get().targetsUsed.size(); i++) {
      averageDistance += Math.abs(result.get().targetsUsed.get(i).getBestCameraToTarget().getX());
    }
    return averageDistance /= (double) result.get().targetsUsed.size();
  }
  
  public double getAverageTagDistanceY(Optional<EstimatedRobotPose> result) {
    double averageDistance = 0;
    for(int i = 0; i < result.get().targetsUsed.size(); i++) {
      averageDistance += Math.abs(result.get().targetsUsed.get(i).getBestCameraToTarget().getY());
    }
    return averageDistance /= (double) result.get().targetsUsed.size();
  }

  public Vector<N3> std(double averageDistanceX, double averageDistanceY) {
    return VecBuilder.fill(xStdMap.get(averageDistanceX), yStdMap.get(averageDistanceY), 9999999); 
    // Make an Interpolating map that uses average distance from camera to find x and y std. 
    // Fill map with varing ranges and stds from aScope
  }

  @Override
  public void periodic() {
    var frontResult = frontCamera.getEstimatedPose();
    var backResult = backCamera.getEstimatedPose();

    /* Front Camera */
    if(!frontResult.isEmpty()) {
      double averageDistance = getAverageTagDistanceX(frontResult);

      if(averageDistance > maxDistance) {
        frontCameraBad = true;
      }

      if(!frontCameraBad) {
        s_Swerve.addVisionMeasurement(frontResult.get().estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(frontResult.get().timestampSeconds), Constants.Vision.TAG_VISION_STDS_FRONT);
      }

      for(int i = 0; i < frontResult.get().targetsUsed.size(); i++) {
        frontTagsUsed.add(frontCamera.getPhotonPoseEstimator().getFieldTags().getTagPose(frontResult.get().targetsUsed.get(i).getFiducialId()).get());
      }

      if(!frontTagsUsed.isEmpty()) {
        Logger.recordOutput("Front Camera Tags Used", frontTagsUsed.toArray(new Pose3d[frontTagsUsed.size()]));
      }
    }
    
    /* Back Camera */
    if(!backResult.isEmpty()) {
      double averageDistance = getAverageTagDistanceX(backResult);

      if(averageDistance > maxDistance) {
        backCameraBad = true;
      }

      if(!backCameraBad) {
        s_Swerve.addVisionMeasurement(backResult.get().estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(backResult.get().timestampSeconds), Constants.Vision.TAG_VISION_STDS_BACK);
      }

      for(int i = 0; i < backResult.get().targetsUsed.size(); i++) {
        backTagsUsed.add(backCamera.getPhotonPoseEstimator().getFieldTags().getTagPose(backResult.get().targetsUsed.get(i).getFiducialId()).get());
      }

      if(!backTagsUsed.isEmpty()) {
        Logger.recordOutput("Back Camera Tags Used", backTagsUsed.toArray(new Pose3d[backTagsUsed.size()]));
      }
    }

    Logger.recordOutput("Front Camera Dead", frontCamera.isDead());
    Logger.recordOutput("Back Camera Dead", backCamera.isDead());
  }
}
