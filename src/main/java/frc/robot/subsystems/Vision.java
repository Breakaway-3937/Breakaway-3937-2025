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
  private final InterpolatingDoubleTreeMap xStdMapFront, yStdMapFront, xStdMapBack, yStdMapBack; //Key = distance, Value = STD
  private boolean frontCameraBad, backCameraBad;
  private boolean xDistanceBad = false, yDistanceBad = false;
  private ArrayList<Pose3d> frontTagsUsed, backTagsUsed;

  /** Creates a new Vision. */
  public Vision(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
    
    try {
      atfl = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);
    } catch(IOException e) {
      e.printStackTrace();
    }

    frontCamera = new BreakaCamera(Constants.Vision.FRONT_CAMERA_NAME, new PhotonPoseEstimator(atfl, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.Vision.FRONT_CAMERA_TRANSFORM));
    backCamera = new BreakaCamera(Constants.Vision.BACK_CAMERA_NAME, new PhotonPoseEstimator(atfl, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.Vision.BACK_CAMERA_TRANSFORM));
  
    xStdMapFront = new InterpolatingDoubleTreeMap();
    yStdMapFront = new InterpolatingDoubleTreeMap();
    xStdMapBack = new InterpolatingDoubleTreeMap();
    yStdMapBack = new InterpolatingDoubleTreeMap();

    frontTagsUsed = new ArrayList<Pose3d>();
    backTagsUsed = new ArrayList<Pose3d>();
  }

  public boolean hasTargets() {
    return frontCamera.getLatest().hasTargets();
  }

  public PhotonPipelineResult getLatestFront() {
    return frontCamera.getLatest();
  }

  public void populateFrontTreeMap() {
    xStdMapFront.put(1.0, 0.0);
    xStdMapFront.put(2.0, 0.0);
    xStdMapFront.put(3.0, 0.0);
    xStdMapFront.put(4.0, 0.0);
    xStdMapFront.put(5.0, 0.0);
    xStdMapFront.put(6.0, 0.0);

    yStdMapFront.put(1.0, 0.0);
    yStdMapFront.put(2.0, 0.0);
    yStdMapFront.put(3.0, 0.0);
    yStdMapFront.put(4.0, 0.0);
    yStdMapFront.put(5.0, 0.0);
    yStdMapFront.put(6.0, 0.0);
  }

  public void populateBackTreeMap() {
    xStdMapBack.put(1.0, 0.0);
    xStdMapBack.put(2.0, 0.0);
    xStdMapBack.put(3.0, 0.0);
    xStdMapBack.put(4.0, 0.0);
    xStdMapBack.put(5.0, 0.0);
    xStdMapBack.put(6.0, 0.0);

    yStdMapBack.put(1.0, 0.0);
    yStdMapBack.put(2.0, 0.0);
    yStdMapBack.put(3.0, 0.0);
    yStdMapBack.put(4.0, 0.0);
    yStdMapBack.put(5.0, 0.0);
    yStdMapBack.put(6.0, 0.0);
  }

  public double getAverageTagDistanceX(Optional<EstimatedRobotPose> result) {
    if(!result.isEmpty()) {
      double averageDistance = 0;
      for(int i = 0; i < result.get().targetsUsed.size(); i++) {
        averageDistance += Math.abs(result.get().targetsUsed.get(i).getBestCameraToTarget().getX());
      }
      xDistanceBad = false;
      return averageDistance /= (double) result.get().targetsUsed.size();
    }
    else {
      xDistanceBad = true;
      return 9999;
    }
  }
  
  public double getAverageTagDistanceY(Optional<EstimatedRobotPose> result) {
    if(!result.isEmpty()) {
      double averageDistance = 0;
      for(int i = 0; i < result.get().targetsUsed.size(); i++) {
        averageDistance += Math.abs(result.get().targetsUsed.get(i).getBestCameraToTarget().getY());
      }
      yDistanceBad = false;
      return averageDistance /= (double) result.get().targetsUsed.size();
    }
    else {
      yDistanceBad = true;
      return 9999;
    }
  }

  public Vector<N3> calcStdFront(double averageDistanceX, double averageDistanceY) {
    return Constants.Vision.TAG_VISION_STDS_FRONT;  //TODO get tree map values then uncomment line below
    //return VecBuilder.fill(xStdMapFront.get(averageDistanceX), yStdMapFront.get(averageDistanceY), 9999999); 
    // Make an Interpolating map that uses average distance from camera to find x and y std. 
    // Fill map with varing ranges and stds from aScope
  }

  public Vector<N3> calcStdBAck(double averageDistanceX, double averageDistanceY) {
    return Constants.Vision.TAG_VISION_STDS_BACK;  //TODO get tree map values then uncomment line below
    //return VecBuilder.fill(xStdMapFront.get(averageDistanceX), yStdMapFront.get(averageDistanceY), 9999999); 
    // Make an Interpolating map that uses average distance from camera to find x and y std. 
    // Fill map with varing ranges and stds from aScope
  }

  @Override
  public void periodic() {
    var frontResult = frontCamera.getEstimatedPose();
    var backResult = backCamera.getEstimatedPose();

    /* Front Camera */
    if(!frontResult.isEmpty()) {
      double averageDistanceX = getAverageTagDistanceX(frontResult);
      double averageDistanceY = getAverageTagDistanceY(frontResult);

      if(averageDistanceX > maxDistance) {
        frontCameraBad = true;
      }

      if(!frontCameraBad) {
        s_Swerve.addVisionMeasurement(frontResult.get().estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(frontResult.get().timestampSeconds), calcStdFront(averageDistanceX, averageDistanceY));
      }

      for(int i = 0; i < frontResult.get().targetsUsed.size(); i++) {
        frontTagsUsed.add(frontCamera.getPhotonPoseEstimator().getFieldTags().getTagPose(frontResult.get().targetsUsed.get(i).getFiducialId()).get());
      }

      if(!frontTagsUsed.isEmpty()) {
        Logger.recordOutput("Vision/Front Camera Tags Used", frontTagsUsed.toArray(new Pose3d[frontTagsUsed.size()]));
      }
    }
    else {
      Logger.recordOutput("Vision/Front Camera Tags Used", new Pose3d[] {});
    }
    
    /* Back Camera */
    if(!backResult.isEmpty()) {
      double averageDistanceX = getAverageTagDistanceX(backResult);
      double averageDistanceY = getAverageTagDistanceY(backResult);

      if(averageDistanceX > maxDistance) {
        backCameraBad = true;
      }

      if(!backCameraBad) {
        s_Swerve.addVisionMeasurement(backResult.get().estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(backResult.get().timestampSeconds), calcStdBAck(averageDistanceX, averageDistanceY));
      }

      for(int i = 0; i < backResult.get().targetsUsed.size(); i++) {
        backTagsUsed.add(backCamera.getPhotonPoseEstimator().getFieldTags().getTagPose(backResult.get().targetsUsed.get(i).getFiducialId()).get());
      }

      if(!backTagsUsed.isEmpty()) {
        Logger.recordOutput("Vision/Back Camera Tags Used", backTagsUsed.toArray(new Pose3d[backTagsUsed.size()]));
      }
    }
    else {
      Logger.recordOutput("Vision/Back Camera Tags Used", new Pose3d[] {});
    }

    Logger.recordOutput("Vision/X Distance Result Empty", xDistanceBad);
    Logger.recordOutput("Vision/Y Distance Result Empty", yDistanceBad);
    Logger.recordOutput("Vision/Front Camera Dead", frontCamera.isDead());
    Logger.recordOutput("Vision/Back Camera Dead", backCamera.isDead());
    Logger.recordOutput("Vision/Front Camera Bad", frontCameraBad);
    Logger.recordOutput("Vision/Back Camera Bad", backCameraBad);
  }
}
