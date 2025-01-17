package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Swerve.AutoPathLocations;

/** Class for acquiring button triggers from Network Tables */
public class OperatorController {
  private static final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private static final NetworkTable table = inst.getTable("OperatorController");

  private static final StringTopic levelTopic = table.getStringTopic("level");
  private static final StringTopic scoringLocationTopic = table.getStringTopic("target");
  private static final StringTopic pickupLocationTopic = table.getStringTopic("station");

  private static final StringEntry levelEntry = levelTopic.getEntry("NO_LEVEL");
  private static final StringEntry scoringLocation = scoringLocationTopic.getEntry("NO_TARGET");
  private static final StringEntry pickupLocation = pickupLocationTopic.getEntry("NO_STATION");

  public static AutoPathLocations getScoringLocation() {
    try {
      return AutoPathLocations.valueOf(scoringLocation.get());
    }
    catch(IllegalArgumentException e) {
      return AutoPathLocations.valueOf("NO_TARGET");
    }
  }

  public static AutoPathLocations getPickUpLocation() {
    System.out.println("I am in the pick up location method");
    try {
      //System.out.println(AutoPathLocations.valueOf(pickupLocation.get()).getLocation());
      return AutoPathLocations.valueOf(pickupLocation.get());
    }
    catch(IllegalArgumentException e) {
      return AutoPathLocations.valueOf("NO_LEVEL");
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
    return new Trigger(() -> scoringLocation.get().equals("CORAL_A"));
  }

  public static Trigger getCoralBTrigger() {
    return new Trigger(() -> scoringLocation.get().equals("CORAL_B"));
  }

  public static Trigger getCoralCTrigger() {
    return new Trigger(() -> scoringLocation.get().equals("CORAL_C"));
  }

  public static Trigger getCoralDTrigger() {
    return new Trigger(() -> scoringLocation.get().equals("CORAL_D"));
  }

  public static Trigger getCoralETrigger() {
    return new Trigger(() -> scoringLocation.get().equals("CORAL_E"));
  }

  public static Trigger getCoralFTrigger() {
    return new Trigger(() -> scoringLocation.get().equals("CORAL_F"));
  }

  public static Trigger getCoralGTrigger() {
    return new Trigger(() -> scoringLocation.get().equals("CORAL_G"));
  }

  public static Trigger getCoralHTrigger() {
    return new Trigger(() -> scoringLocation.get().equals("CORAL_H"));
  }

  public static Trigger getCoralITrigger() {
    return new Trigger(() -> scoringLocation.get().equals("CORAL_I"));
  }

  public static Trigger getCoralJTrigger() {
    return new Trigger(() -> scoringLocation.get().equals("CORAL_J"));
  }

  public static Trigger getCoralKTrigger() {
    return new Trigger(() -> scoringLocation.get().equals("CORAL_K"));
  }

  public static Trigger getCoralLTrigger() {
    return new Trigger(() -> scoringLocation.get().equals("CORAL_L"));
  }

  public static Trigger getNoTargetTrigger() {
    return new Trigger(() -> scoringLocation.get().equals("NO_TARGET"));
  }
}
