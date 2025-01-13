package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class OperatorController {
  private static final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private static final NetworkTable table = inst.getTable("OperatorController");
  private static final StringTopic topic = table.getStringTopic("Jeffords");
  private static final StringSubscriber stringSub = topic.subscribe("");

  public static Trigger getLevelOneTrigger() {
    return new Trigger(() -> stringSub.get().equals("Level 1"));
  }

  public static Trigger getLevelTwoTrigger() {
    return new Trigger(() -> stringSub.get().equals("Level 2"));
  }

  public static Trigger getLevelThreeTrigger() {
    return new Trigger(() -> stringSub.get().equals("Level 3"));
  }

  public static Trigger getLevelFourTrigger() {
    return new Trigger(() -> stringSub.get().equals("Level 4"));
  }

  public static Trigger getStopTrigger() {
    return new Trigger(() -> stringSub.get().equals("Stop"));
  }
}
