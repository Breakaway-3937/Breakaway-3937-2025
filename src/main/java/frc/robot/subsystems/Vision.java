// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
  private AprilTagFieldLayout atfl;

  /** Creates a new Vision. */
  public Vision() {
    try {
      atfl = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile); //FIXME 2025
    }
    catch(IOException e){}
    
  }

  //Pass joystick/buttons to method. Read from both to determine how to rotate. 
  //Prevents gaint stupid commmands. Or make multiple seperat commands
  public double getRotationSpeed(Joystick rotationJoysitck, JoystickButton rotationButton) {

    if(rotationButton.getAsBoolean()) {
      //camera.getLatest...
      //More code goes here
      return 0.0;
    } 
    //More Logic for other line ups. 
    else {
      return rotationJoysitck.getRawAxis(Constants.Controllers.ROTATION_AXIS);
    }
  }

  @Override
  public void periodic() {}

}