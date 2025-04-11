// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import java.io.IOException;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.lib.Vision.BreakaCamera;
import frc.robot.subsystems.Swerve.Swerve;

public class Vision extends SubsystemBase {
  private AprilTagFieldLayout atfl;
  private final BreakaCamera frontCamera;
  private final BreakaCamera backLeftCamera;
  private final BreakaCamera backRightCamera;
  private final PhoenixPIDController rotationController;
  private final Swerve s_Swerve;
  private final double maxDistance = 5; // In meters
  private boolean frontCameraBad, backLeftCameraBad, backRightCameraBad;
  private boolean xDistanceBad = false, noBack = false;
  private int[] blueTags = {17, 18, 19, 20, 21, 22};
  private int[] redTags = {6, 7, 8, 9, 10, 11};
  private final InterpolatingDoubleTreeMap tagsStds;

  /** Creates a new Vision. */
  public Vision(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
    
    try {
      atfl = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);
    } catch(IOException e) {
      e.printStackTrace();
    }

    frontCamera = new BreakaCamera(Constants.Vision.FRONT_CAMERA_NAME, new PhotonPoseEstimator(atfl, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.Vision.FRONT_CAMERA_TRANSFORM));
    backLeftCamera = new BreakaCamera(Constants.Vision.BACK_LEFT_CAMERA_NAME, new PhotonPoseEstimator(atfl, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.Vision.BACK_LEFT_CAMERA_TRANSFORM));
    backRightCamera = new BreakaCamera(Constants.Vision.BACK_RIGHT_CAMERA_NAME, new PhotonPoseEstimator(atfl, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.Vision.BACK_RIGHT_CAMERA_TRANSFORM));

    rotationController = new PhoenixPIDController(4.5, 0, 0);
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    rotationController.setTolerance(0.1);

    tagsStds = new InterpolatingDoubleTreeMap();
    tagsStds.put(0.752358, 0.005);
    tagsStds.put(1.016358, 0.0135);
    tagsStds.put(1.296358, 0.016);
    tagsStds.put(1.574358, 0.038);
    tagsStds.put(1.913358, 0.0515);
    tagsStds.put(2.184358, 0.0925);
    tagsStds.put(2.493358, 0.12);
    tagsStds.put(2.758358, 0.14);
    tagsStds.put(3.223358, 0.17);
    tagsStds.put(4.093358, 0.27);
    tagsStds.put(4.726358, 0.38);

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
  
  public boolean badTags(Optional<EstimatedRobotPose> result) {
    boolean bad = false;

    if(!result.isEmpty()) {
      for(int i = 0; i < result.get().targetsUsed.size(); i++) {
        int tagUsed = result.get().targetsUsed.get(i).fiducialId;
        if(tagUsed == 14 || tagUsed == 15 || tagUsed == 4 || tagUsed == 5 || tagUsed == 16 || tagUsed == 3 || tagUsed == 13 || tagUsed == 12 || tagUsed == 2 || tagUsed == 1) {
          bad = true;
        }
      }

      var allianceColor = DriverStation.getAlliance();

      if(allianceColor.isPresent()) {
        if(allianceColor.get().equals(Alliance.Blue)) {
          for(int i = 0; i < result.get().targetsUsed.size(); i++) {
            for(int j = 0; j < redTags.length; j++) {
              if(result.get().targetsUsed.get(i).getFiducialId() == redTags[j]) {
                bad = true;
              }
            }
          }
        }

        if(allianceColor.get().equals(Alliance.Red)) {
          for(int i = 0; i < result.get().targetsUsed.size(); i++) {
            for(int j = 0; j < blueTags.length; j++) {
              if(result.get().targetsUsed.get(i).getFiducialId() == blueTags[j]) {
                bad = true;
              }
            }
          }
        }
      }
    }
    return bad;
  }

  public BooleanSupplier coralAlignTooFar() {
    return () -> {
      var result = frontCamera.getEstimatedPose();
      if(result.isPresent()) {
        return getAverageTagDistanceX(result) > 0.3;
      }
      else {
        return false;
      }
    };
  }

  public Vector<N3> calcStd(double distance) {
    double xy = tagsStds.get(distance) * 2.25;
    return VecBuilder.fill(xy, xy, 99999);
  }

  public BooleanSupplier funeral() {
    return () -> frontCamera.isDead() || backLeftCamera.isDead() || backRightCamera.isDead();
  }

  public Command refuseBack() {
    return runOnce(() -> noBack = true);
  }

  public Command unrefuseBack() {
    return runOnce(() -> noBack = false);
  }

  @Override
  public void periodic() {
    var frontResult = frontCamera.getEstimatedPose();
    var backLeftResult = backLeftCamera.getEstimatedPose();
    var backRightResult = backRightCamera.getEstimatedPose();

    /* Front Camera */
    if(!frontResult.isEmpty()) {
      double averageDistanceX = getAverageTagDistanceX(frontResult);
      

      if(averageDistanceX > maxDistance) {
        frontCameraBad = true;
      }

      if(badTags(frontResult)) {
        frontCameraBad = true;
      }
        
      if(!frontCameraBad) {
        Pose2d pose = new Pose2d(frontResult.get().estimatedPose.getX(), frontResult.get().estimatedPose.getY(), s_Swerve.getState().Pose.getRotation());
        if(!(DriverStation.isAutonomousEnabled() && (s_Swerve.getState().Pose.getX() < 1.7 || s_Swerve.getState().Pose.getX() > 14.8))) {
          SmartDashboard.putNumberArray("F Tag std", calcStd(averageDistanceX).getData());
          s_Swerve.addVisionMeasurement(pose, Utils.fpgaToCurrentTime(frontResult.get().timestampSeconds), calcStd(averageDistanceX));
        }
      }
    }
    
    /* Back Left Camera */
    if(!backLeftResult.isEmpty() && (frontCameraBad || frontResult.isEmpty()) && !noBack) {
      double averageDistanceX = getAverageTagDistanceX(backLeftResult);

      if(averageDistanceX > maxDistance) {
        backLeftCameraBad = true;
      }

      if(badTags(backLeftResult) || averageDistanceX < Feet.of(1).in(Meters)) {
        backLeftCameraBad = true;
      }

      if(!backLeftCameraBad && DriverStation.isTeleopEnabled()) {
        Pose2d pose = new Pose2d(backLeftResult.get().estimatedPose.getX(), backLeftResult.get().estimatedPose.getY(), s_Swerve.getState().Pose.getRotation());
        SmartDashboard.putNumberArray(" BF Tag std", calcStd(averageDistanceX).getData());
        s_Swerve.addVisionMeasurement(pose, Utils.fpgaToCurrentTime(backLeftResult.get().timestampSeconds), calcStd(averageDistanceX));
      }
    }

    /* Back Right Camera */
    if(!backRightResult.isEmpty() && (frontCameraBad || frontResult.isEmpty() || backLeftCameraBad || backLeftResult.isEmpty()) && !noBack) {
      double averageDistanceX = getAverageTagDistanceX(backRightResult);

      if(averageDistanceX > maxDistance || averageDistanceX < Feet.of(1).in(Meters)) {
        backRightCameraBad = true;
      }

      if(badTags(backRightResult)) {
        backRightCameraBad = true;
      }

      if(!backRightCameraBad && DriverStation.isTeleopEnabled()) {
        Pose2d pose = new Pose2d(backRightResult.get().estimatedPose.getX(), backRightResult.get().estimatedPose.getY(), s_Swerve.getState().Pose.getRotation());
        SmartDashboard.putNumberArray("BR Tag std", calcStd(averageDistanceX).getData());
        s_Swerve.addVisionMeasurement(pose, Utils.fpgaToCurrentTime(backRightResult.get().timestampSeconds), calcStd(averageDistanceX));
      }
    }

    Logger.recordOutput("Vision/X Distance Result Empty", xDistanceBad);
    Logger.recordOutput("Vision/Front Camera Dead", frontCamera.isDead());
    Logger.recordOutput("Vision/Back Left Camera Dead", backLeftCamera.isDead());
    Logger.recordOutput("Vision/Back Right Camera Dead", backRightCamera.isDead());
    Logger.recordOutput("Vision/Front Camera Bad", frontCameraBad);
    Logger.recordOutput("Vision/Back Left Camera Bad", backLeftCameraBad);
    Logger.recordOutput("Vision/Back Right Camera Bad", backRightCameraBad);

    frontCameraBad = false;
    backLeftCameraBad = false;
    backRightCameraBad = false;
  }
}
