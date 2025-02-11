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

    frontCamera = new BreakaCamera("FrontCamera", new PhotonPoseEstimator(atfl, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.Vision.FRONT_CAMERA_TRANSFORM));
    backCamera = new BreakaCamera("3937", new PhotonPoseEstimator(atfl, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.Vision.BACK_CAMERA_TRANSFORM));
  }

  public double getX() {
    return frontCamera.getLatest().hasTargets() ? frontCamera.getLatest().getBestTarget().getBestCameraToTarget().getX() : 0;
  }

  public boolean hasTargets() {
    return frontCamera.getLatest().hasTargets();
  }

  public PhotonPipelineResult getLatestFront() {
    return frontCamera.getLatest();
  }

  public double getAverageTagDistance(Optional<EstimatedRobotPose> result) {
    double averageDistance = 0;
    for(int i = 0; i < result.get().targetsUsed.size(); i++) {
      averageDistance += Math.abs(result.get().targetsUsed.get(i).getBestCameraToTarget().getX());
    }
    return averageDistance /= (double) result.get().targetsUsed.size();
  }

  public Vector<N3> std(double averageDistance) {
    return VecBuilder.fill(0, 0, 0); 
    // Make an Interpolating map that uses average distance from camera to find x and y std. 
    // Fill map with varing ranges and stds from aScope
  }

  @Override
  public void periodic() {
    var result = frontCamera.getEstimatedPose();
    var otherResult = backCamera.getEstimatedPose();

    /* Front Camera */
    if(!result.isEmpty()) {
      double averageDistance = getAverageTagDistance(result);

      if(averageDistance > maxDistance) {
        frontCameraBad = true;
      }

      if(!frontCameraBad) {
        s_Swerve.addVisionMeasurement(result.get().estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(result.get().timestampSeconds), Constants.Vision.TAG_VISION_STDS_FRONT);
      }

      for(int i = 0; i < result.get().targetsUsed.size(); i++) {
        frontTagsUsed.add(frontCamera.getPhotonPoseEstimator().getFieldTags().getTagPose(result.get().targetsUsed.get(i).getFiducialId()).get());
      }

      if(!frontTagsUsed.isEmpty()) {
        Logger.recordOutput("Front Camera Tags Used", frontTagsUsed.toArray(new Pose3d[frontTagsUsed.size()]));
      }
    }
    /* Back Camera */
    if(!otherResult.isEmpty()) {
      s_Swerve.addVisionMeasurement(otherResult.get().estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(otherResult.get().timestampSeconds), Constants.Vision.TAG_VISION_STDS_FRONT);
    }

    Logger.recordOutput("Front Camera Dead", frontCamera.isDead());
    Logger.recordOutput("Back Camera Dead", backCamera.isDead());
  }
}
