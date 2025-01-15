package frc.robot.subsystems.Swerve;


public enum AutoPathLocations {
    CORAL_ONE("CORAL_ONE"),
    CORAL_TWO(""),
    CORAL_THREE(""),
    L4("L4 Right #4");

    private final String path;

    private AutoPathLocations(String path) {
        this.path = path;
    }

    public String getPath() {
        return path;
    }
}
