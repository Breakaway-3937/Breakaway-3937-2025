package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.StringTopic;
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

  public static Supplier<AutoPathLocations> getScoringLocation() {
    try {
      return () -> AutoPathLocations.valueOf(scoringLocation.get());
    }
    catch(IllegalArgumentException e) {
      return () -> AutoPathLocations.valueOf("NO_TARGET");
    }
  }

  public static Supplier<AutoPathLocations> getPickUpLocation() {
    try {
      return () -> AutoPathLocations.valueOf(pickupLocation.get());
    }
    catch(IllegalArgumentException e) {
      return () -> AutoPathLocations.valueOf("NO_LEVEL");
    }
  }

  public static String getLevel() {
    return levelEntry.get();
  }
}
