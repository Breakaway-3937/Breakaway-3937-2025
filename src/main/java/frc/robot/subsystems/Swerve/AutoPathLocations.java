package frc.robot.subsystems.Swerve;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.PathPlannerPath;

public enum AutoPathLocations {
    //TODO: Update with new values.
    CORAL_A(breakaParse("A")),
    CORAL_B(breakaParse("B")),
    CORAL_C(breakaParse("C")),
    CORAL_D(breakaParse("D")),
    CORAL_E(breakaParse("E")),
    CORAL_F(breakaParse("F")),
    CORAL_G(breakaParse("G")),
    CORAL_H(breakaParse("H")),
    CORAL_I(breakaParse("I")),
    CORAL_J(breakaParse("J")),
    CORAL_K(breakaParse("K")),
    CORAL_L(breakaParse("L")),
    NO_TARGET(null),
    STATION_ONE(breakaParse("words")),
    STATION_TWO(breakaParse("words")),
    STATION_THREE(breakaParse("words")),
    STATION_FOUR(breakaParse("words")),
    STATION_FIVE(breakaParse("words")),
    STATION_SIX(breakaParse("words")),
    NO_STATION(null);

    private final PathPlannerPath path;

    private AutoPathLocations(PathPlannerPath path) {
        this.path = path;
    }

   public PathPlannerPath getPath() {
        return path;
   }

   private static PathPlannerPath breakaParse(String pathName) {
        try {
            return PathPlannerPath.fromPathFile(pathName);
        } 
        catch (Exception e) {
            Logger.recordOutput("Path File Alert", pathName);
            return null;
        }
   }
}
