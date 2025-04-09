// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.lib.Vision.BreakaCamera;
import frc.robot.lib.Vision.LimeAway;
import frc.robot.subsystems.Swerve.Swerve;

public class Vision extends SubsystemBase {
  private AprilTagFieldLayout atfl;
  private final BreakaCamera frontCamera;
  private final BreakaCamera backLeftCamera;
  private final BreakaCamera backRightCamera;
  //private final LimeAway coralCamera;
  private final PhoenixPIDController rotationController;
  private final Swerve s_Swerve;
  private final double maxDistance = 5; // In meters
  private boolean frontCameraBad, backCameraBad;
  private boolean xDistanceBad = false, noBack = false;
  private int[] blueTags = {17, 18, 19, 20, 21, 22};
  private int[] redTags = {6, 7, 8, 9, 10, 11}; 

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
    backRightCamera = new BreakaCamera(Constants.Vision.BACK_LEFT_CAMERA_NAME, new PhotonPoseEstimator(atfl, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.Vision.BACK_LEFT_CAMERA_TRANSFORM));
    //coralCamera = new LimeAway(Constants.Vision.CORAL_CAMERA_NAME);

    rotationController = new PhoenixPIDController(4.5, 0, 0);
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    rotationController.setTolerance(0.1);

    //coralCamera.turnLedsOff();
  }

  public boolean hasFrontTargets() {
    return frontCamera.getLatest().hasTargets();
  }

  public PhotonPipelineResult getLatestFront() {
    return frontCamera.getLatest();
  }

  /*public double getCoralTargetSpeed() {
    if(coralCamera.hasTarget()) {
      //If setpoint changes needs to be in radians
      double speed = rotationController.calculate(coralCamera.getTX().in(Radians), 0, Utils.getCurrentTimeSeconds());

      if (speed > Constants.Swerve.MAX_ANGULAR_RATE) {
        speed = Constants.Swerve.MAX_ANGULAR_RATE;
      } 
      else if (speed < -Constants.Swerve.MAX_ANGULAR_RATE) {
        speed = -Constants.Swerve.MAX_ANGULAR_RATE;
      }

      return speed;
    }
    else {
      return 0;
    }
  }*/

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
        if(tagUsed == 14 || tagUsed == 15 || tagUsed == 4 || tagUsed == 5 || tagUsed == 16 || tagUsed == 3) {
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

  public BooleanSupplier funeral() {
    return () -> frontCamera.isDead() || backLeftCamera.isDead();
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
    var backResult = backLeftCamera.getEstimatedPose();

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
          s_Swerve.addVisionMeasurement(pose, Utils.fpgaToCurrentTime(frontResult.get().timestampSeconds), Constants.Vision.TAG_VISION_STDS);
        }
      }
    }
    
    /* Back Camera */
    if(!backResult.isEmpty() && (frontCameraBad || frontResult.isEmpty()) && !noBack) {
      double averageDistanceX = getAverageTagDistanceX(backResult);

      if(averageDistanceX > maxDistance) {
        backCameraBad = true;
      }

      if(badTags(backResult)) {
        backCameraBad = true;
      }

      if(!backCameraBad && DriverStation.isTeleopEnabled()) {
        Pose2d pose = new Pose2d(backResult.get().estimatedPose.getX(), backResult.get().estimatedPose.getY(), s_Swerve.getState().Pose.getRotation());
        s_Swerve.addVisionMeasurement(pose, Utils.fpgaToCurrentTime(backResult.get().timestampSeconds), Constants.Vision.TAG_VISION_STDS);
      }
    }

    //Logger.recordOutput("Coral Rotation Speed", getCoralTargetSpeed());
    Logger.recordOutput("Vision/X Distance Result Empty", xDistanceBad);
    Logger.recordOutput("Vision/Front Camera Dead", frontCamera.isDead());
    Logger.recordOutput("Vision/Back Camera Dead", backLeftCamera.isDead());
    Logger.recordOutput("Vision/Front Camera Bad", frontCameraBad);
    Logger.recordOutput("Vision/Back Camera Bad", backCameraBad);

    frontCameraBad = false;
    backCameraBad = false;

    /*if(Constants.DEBUG) {
      SmartDashboard.putString("Detector Class", coralCamera.getDetectorClass());
      SmartDashboard.putNumber("Detector Index", coralCamera.getDetectorIndex());
      SmartDashboard.putNumber("Area Of Target", coralCamera.getAreaOfTarget());
      SmartDashboard.putNumber("TX", coralCamera.getTX().in(Degrees));
      SmartDashboard.putNumber("TY", coralCamera.getTY().in(Degrees));
    }*/
  }
}
