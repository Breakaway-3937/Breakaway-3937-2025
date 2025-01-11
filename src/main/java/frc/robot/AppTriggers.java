// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class AppTriggers {
  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable table;
  private StringTopic topic;
  private StringSubscriber stringSub;

  public AppTriggers() {
    table = inst.getTable("OperatorController");
    topic = table.getStringTopic("Jeffords");
    stringSub = topic.subscribe("");
  }

  public Trigger getlevelOneTrigger() {
    return new Trigger(() -> stringSub.get().equals("Level 1"));
  }

  public Trigger getlevelTwoTrigger() {
    return new Trigger(() -> stringSub.get().equals("Level 2"));
  }

  public Trigger getlevelThreeTrigger() {
    return new Trigger(() -> stringSub.get().equals("Level 3"));
  }

  public Trigger getlevelFourTrigger() {
    return new Trigger(() -> stringSub.get().equals("Level 4"));
  }

  public Trigger stop() {
    return new Trigger(() -> stringSub.get().equals("Stop"));
  }

}
