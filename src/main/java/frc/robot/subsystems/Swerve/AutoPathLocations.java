package frc.robot.subsystems.Swerve;

import org.littletonrobotics.junction.Logger;
import com.pathplanner.lib.path.PathPlannerPath;

public enum AutoPathLocations {
    
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
    CORAL_A_BACKWARDS(breakaParse("Backwards A")),
    CORAL_B_BACKWARDS(breakaParse("Backwards B")),
    CORAL_C_BACKWARDS(breakaParse("Backwards C")),
    CORAL_D_BACKWARDS(breakaParse("Backwards D")),
    CORAL_E_BACKWARDS(breakaParse("Backwards E")),
    CORAL_F_BACKWARDS(breakaParse("Backwards F")),
    CORAL_G_BACKWARDS(breakaParse("Backwards G")),
    CORAL_H_BACKWARDS(breakaParse("Backwards H")),
    CORAL_I_BACKWARDS(breakaParse("Backwards I")),
    CORAL_J_BACKWARDS(breakaParse("Backwards J")),
    CORAL_K_BACKWARDS(breakaParse("Backwards K")),
    CORAL_L_BACKWARDS(breakaParse("Backwards L"));

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
            System.out.println(e);
            return null;
        }
   }
}
