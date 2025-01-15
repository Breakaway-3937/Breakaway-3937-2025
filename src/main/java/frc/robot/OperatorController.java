package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Swerve.AutoPathLocations;

/** Class for acquiring button triggers from Network Tables */
public class OperatorController {
  private static final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private static final NetworkTable table = inst.getTable("OperatorController");
  private static final StringTopic level = table.getStringTopic("level");
  private static final StringTopic target = table.getStringTopic("target");
  private static final StringEntry levelEntry= level.getEntry("NO_LEVEL");
  private static final StringEntry targeEntry = target.getEntry("NO_TARGET");

  public static AutoPathLocations getLocation() {
    try {
      return AutoPathLocations.valueOf(targeEntry.get());
    }
    catch(IllegalArgumentException e) {
      return AutoPathLocations.valueOf("NO_TARGET");
    }
  }

  public static AutoPathLocations getLevel() {
    try {
      return AutoPathLocations.valueOf(levelEntry.get());
    }
    catch(IllegalArgumentException e) {
      return AutoPathLocations.valueOf("NO_LEVEL");
    }
  }

  public static Trigger getLevelOneTrigger() {
    return new Trigger(() -> levelEntry.get().equals("LEVEL_ONE"));
  }

  public static Trigger getLevelTwoTrigger() {
    return new Trigger(() -> levelEntry.get().equals("LEVEL_TWO"));
  }

  public static Trigger getLevelThreeTrigger() {
    return new Trigger(() -> levelEntry.get().equals("LEVEL_THREE"));
  }

  public static Trigger getLevelFourTrigger() {
    return new Trigger(() -> levelEntry.get().equals("LEVEL_TWO"));
  }

  public static Trigger getNoLevelTrigger() {
    return new Trigger(() -> levelEntry.get().equals("NO_LEVEL"));
  }

  public static Trigger getCoralATrigger() {
    return new Trigger(() -> targeEntry.get().equals("CORAL_A"));
  }

  public static Trigger getCoralBTrigger() {
    return new Trigger(() -> targeEntry.get().equals("CORAL_B"));
  }

  public static Trigger getCoralCTrigger() {
    return new Trigger(() -> targeEntry.get().equals("CORAL_C"));
  }

  public static Trigger getCoralDTrigger() {
    return new Trigger(() -> targeEntry.get().equals("CORAL_D"));
  }

  public static Trigger getCoralETrigger() {
    return new Trigger(() -> targeEntry.get().equals("CORAL_E"));
  }

  public static Trigger getCoralFTrigger() {
    return new Trigger(() -> targeEntry.get().equals("CORAL_F"));
  }

  public static Trigger getCoralGTrigger() {
    return new Trigger(() -> targeEntry.get().equals("CORAL_G"));
  }

  public static Trigger getCoralHTrigger() {
    return new Trigger(() -> targeEntry.get().equals("CORAL_H"));
  }

  public static Trigger getCoralITrigger() {
    return new Trigger(() -> targeEntry.get().equals("CORAL_I"));
  }

  public static Trigger getCoralJTrigger() {
    return new Trigger(() -> targeEntry.get().equals("CORAL_J"));
  }

  public static Trigger getCoralKTrigger() {
    return new Trigger(() -> targeEntry.get().equals("CORAL_K"));
  }

  public static Trigger getCoralLTrigger() {
    return new Trigger(() -> targeEntry.get().equals("CORAL_L"));
  }

  public static Trigger getNoTargetTrigger() {
    return new Trigger(() -> targeEntry.get().equals("NO_TARGET"));
  }
}
