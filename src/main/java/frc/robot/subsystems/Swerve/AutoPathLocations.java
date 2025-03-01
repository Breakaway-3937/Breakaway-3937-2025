package frc.robot.subsystems.Swerve;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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
    STATION_ONE(breakaParse("Station Left")),
    STATION_TWO(breakaParse("Station Left")),
    STATION_THREE(breakaParse("Station Left")),
    STATION_FOUR(breakaParse("Station Right")),
    STATION_FIVE(breakaParse("Station Right")),
    STATION_SIX(breakaParse("Station Right")),
    NO_STATION(null);

    private final PathPlannerPath path;

    private AutoPathLocations(PathPlannerPath path) {
        this.path = path;
    }

   public PathPlannerPath getPath() {
        return path;
   }

   /*public double getYGoal() {
        if(!path.getPathPoses().isEmpty()) {
            return path.getAllPathPoints().get(path.getAllPathPoints().size() - 1).position.getY();
        }
        else {
            return 0.0; //TODO get y
        }
   }*/

   private static PathPlannerPath breakaParse(String pathName) {
        try {
            if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                return PathPlannerPath.fromPathFile(pathName).flipPath();
            }
            else {
                return PathPlannerPath.fromPathFile(pathName);
            }
        } 
        catch (Exception e) {
            Logger.recordOutput("Path File Alert", pathName);
            return null;
        }
   }
}
