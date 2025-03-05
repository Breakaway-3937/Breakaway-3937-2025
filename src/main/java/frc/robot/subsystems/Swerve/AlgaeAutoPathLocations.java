package frc.robot.subsystems.Swerve;

import org.littletonrobotics.junction.Logger;
import com.pathplanner.lib.path.PathPlannerPath;

public enum AlgaeAutoPathLocations {
    
    ALGAE_AB(breakaParse("ALGAE AB")),
    ALGAE_CD(breakaParse("ALGAE CD")),
    ALGAE_EF(breakaParse("ALGAE EF")),
    ALGAE_GH(breakaParse("ALGAE GH")),
    ALGAE_IJ(breakaParse("ALGAE IJ")),
    ALGAE_KL(breakaParse("ALGAE KL"));

    private final PathPlannerPath path;

    private AlgaeAutoPathLocations(PathPlannerPath path) {
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
