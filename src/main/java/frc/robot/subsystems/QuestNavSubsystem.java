// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.QuestNav;
import frc.robot.subsystems.Swerve.Swerve;

public class QuestNavSubsystem extends SubsystemBase {

  public QuestNav questNav = new QuestNav();
  private Transform2d QUEST_TO_ROBOT = new Transform2d(0.27,0.0, new Rotation2d());
  private Swerve s_Swerve;
  
  public QuestNavSubsystem(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
  }

  public void setQuestNavPose(Pose2d pose) {
    questNav.setPose(pose.transformBy(QUEST_TO_ROBOT));
  }

  public Pose2d getPose() {
    return questNav.getPose().transformBy(QUEST_TO_ROBOT.inverse());
  }

  public void updateVisionMeasurement() {
    Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(0.02, 0.02, 0.035); //The suggested Standerd Deviations for QuestNav

    if (questNav.isConnected() && questNav.isTracking()) {

        Pose2d pose = getPose();

        // Get timestamp from the QuestNav instance
        double timestamp = questNav.getTimestamp();

        // Convert FPGA timestamp to CTRE's time domain using Phoenix 6 utility
        double ctreTimestamp = Utils.fpgaToCurrentTime(timestamp);

        // Add the measurement to our estimator
        s_Swerve.addVisionMeasurement(pose, ctreTimestamp, QUESTNAV_STD_DEVS);
    }
  }

  @Override
  public void periodic() {
    questNav.commandPeriodic(); 
    updateVisionMeasurement();
  }
}
