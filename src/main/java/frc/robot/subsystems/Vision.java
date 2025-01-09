// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.lib.Vision.BreakaCamera;

public class Vision extends SubsystemBase {
  private AprilTagFieldLayout atfl;
  private BreakaCamera camera;
  private Spark spark;

  /** Creates a new Vision. */
  public Vision() {
    try {
      atfl = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile); //FIXME 2025
    }
    catch(IOException e){}

    camera = new BreakaCamera("getName()", new PhotonPoseEstimator(atfl, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d()));
    spark = new Spark(0);
    
  }


  public Command motorOn() {
    return run(() -> spark.set(1));
  }

  @Override
  public void periodic() {}

}