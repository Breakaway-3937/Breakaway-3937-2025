// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.Vision.BreakaCamera;
import frc.robot.subsystems.Swerve.Swerve;

public class Vision extends SubsystemBase {
  private AprilTagFieldLayout atfl;
  private BreakaCamera camera;
  private Swerve swerve;
  private Field2d field;
  public Pose2d result1;
  public double result2;

  /** Creates a new Vision. */
  public Vision(Swerve swerve) {
    this.swerve = swerve;
    try {
      atfl = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
    } catch(IOException e) {
      e.printStackTrace();
    }

    field = new Field2d();

    SmartDashboard.putData(field);

    camera = new BreakaCamera("FrontCamera", new PhotonPoseEstimator(atfl, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.Vision.FRONT_CAMERA_TRANSFORM));    
  }

  @Override
  public void periodic() {
    var result = camera.getEstimatedPose();
    if(!result.isEmpty()) {
      System.out.println("UPDATE");
      result1 = result.get().estimatedPose.toPose2d();
      result2 = result.get().timestampSeconds;
      field.setRobotPose(result.get().estimatedPose.toPose2d());
      swerve.addVisionMeasurement(result.get().estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(result.get().timestampSeconds), Constants.Vision.TAG_VISION_STDS_FRONT);
    }
  }

}