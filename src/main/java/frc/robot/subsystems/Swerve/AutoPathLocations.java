package frc.robot.subsystems.Swerve;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.PathPlannerPath;

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
    ALGAE_AB(breakaParse("ALGAE AB")),
    ALGAE_CD(breakaParse("ALGAE CD")),
    ALGAE_EF(breakaParse("ALGAE EF")),
    ALGAE_GH(breakaParse("ALGAE GH")),
    ALGAE_IJ(breakaParse("ALGAE IJ")),
    ALGAE_KL(breakaParse("ALGAE KL")),
    NO_TARGET(null);

    private final PathPlannerPath path;

    private AutoPathLocations(PathPlannerPath path) {
        this.path = path;
    }

   public PathPlannerPath getPath() {
        return path;
   }

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
