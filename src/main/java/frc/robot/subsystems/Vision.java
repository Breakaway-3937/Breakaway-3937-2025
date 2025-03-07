// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.lib.Vision.BreakaCamera;
import frc.robot.subsystems.Swerve.Swerve;

public class Vision extends SubsystemBase {
  private AprilTagFieldLayout atfl;
  private final BreakaCamera frontCamera;
  private final BreakaCamera backCamera;
  private final Swerve s_Swerve;
  private final double maxDistance = 6; // In meters
  private boolean frontCameraBad, backCameraBad;
  private boolean xDistanceBad = false, yDistanceBad = false;
  private final ArrayList<Pose3d> frontTagsUsed, backTagsUsed;

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

    frontTagsUsed = new ArrayList<Pose3d>();
    backTagsUsed = new ArrayList<Pose3d>();
  }

  public boolean hasFrontTargets() {
    return frontCamera.getLatest().hasTargets();
  }

  public PhotonPipelineResult getLatestFront() {
    return frontCamera.getLatest();
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

  public BooleanSupplier funeral() {
    return () -> frontCamera.isDead() || backCamera.isDead();
  }

  @Override
  public void periodic() {
    var frontResult = frontCamera.getEstimatedPose();
    var backResult = backCamera.getEstimatedPose();

    /* Front Camera */
    if(!frontResult.isEmpty()) {
      double averageDistanceX = getAverageTagDistanceX(frontResult);

      if(averageDistanceX > maxDistance) {
        frontCameraBad = true;
      }

      if(!frontCameraBad) {
        Pose2d pose = new Pose2d(frontResult.get().estimatedPose.getX(), frontResult.get().estimatedPose.getY(), s_Swerve.getState().Pose.getRotation());
        if(!(DriverStation.isAutonomousEnabled() && (s_Swerve.getState().Pose.getX() < 1.7 || s_Swerve.getState().Pose.getX() > 14.8))) {
          s_Swerve.addVisionMeasurement(pose, Utils.fpgaToCurrentTime(frontResult.get().timestampSeconds), Constants.Vision.TAG_VISION_STDS);
        }
      }

      for(int i = 0; i < frontResult.get().targetsUsed.size(); i++) {
        frontTagsUsed.add(frontCamera.getPhotonPoseEstimator().getFieldTags().getTagPose(frontResult.get().targetsUsed.get(i).getFiducialId()).get());
      }

      if(!frontTagsUsed.isEmpty()) {
        Logger.recordOutput("Vision/Front Camera Tags Used", frontTagsUsed.toArray(new Pose3d[frontTagsUsed.size()]));
      }
    }
    else {
      frontTagsUsed.clear();
    }
    
    /* Back Camera */
    if(!backResult.isEmpty() && (frontCameraBad || frontResult.isEmpty())) {
      double averageDistanceX = getAverageTagDistanceX(backResult);

      if(averageDistanceX > maxDistance) {
        backCameraBad = true;
      }

      if(!backCameraBad && DriverStation.isTeleopEnabled()) {
        Pose2d pose = new Pose2d(backResult.get().estimatedPose.getX(), backResult.get().estimatedPose.getY(), s_Swerve.getState().Pose.getRotation());
        s_Swerve.addVisionMeasurement(pose, Utils.fpgaToCurrentTime(backResult.get().timestampSeconds), Constants.Vision.TAG_VISION_STDS);
      }

      for(int i = 0; i < backResult.get().targetsUsed.size(); i++) {
        backTagsUsed.add(backCamera.getPhotonPoseEstimator().getFieldTags().getTagPose(backResult.get().targetsUsed.get(i).getFiducialId()).get());
      }

      if(!backTagsUsed.isEmpty()) {
        Logger.recordOutput("Vision/Back Camera Tags Used", backTagsUsed.toArray(new Pose3d[backTagsUsed.size()]));
      }
    }
    else {
      backTagsUsed.clear();
    }

    Logger.recordOutput("Vision/X Distance Result Empty", xDistanceBad);
    Logger.recordOutput("Vision/Y Distance Result Empty", yDistanceBad);
    Logger.recordOutput("Vision/Front Camera Dead", frontCamera.isDead());
    Logger.recordOutput("Vision/Back Camera Dead", backCamera.isDead());
    Logger.recordOutput("Vision/Front Camera Bad", frontCameraBad);
    Logger.recordOutput("Vision/Back Camera Bad", backCameraBad);

    frontCameraBad = false;
    backCameraBad = false;
  }
}
