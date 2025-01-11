package frc.robot.subsystems.Swerve;


public enum AutoPathLocations {
    CORAL_ONE("CORAL_ONE"),
    CORAL_TWO(""),
    CORAL_THREE("");

    private final String path;

    private AutoPathLocations(String path) {
        this.path = path;
    }

    public String getPath() {
        return path;
    }
}
