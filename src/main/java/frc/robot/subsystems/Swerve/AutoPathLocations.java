package frc.robot.subsystems.Swerve;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public enum AutoPathLocations {
    CORAL_A(new Pose2d(3.062, 4.2, Rotation2d.fromDegrees(0))),
    CORAL_B(new Pose2d(3.05, 3.85, Rotation2d.fromDegrees(0))),
    CORAL_C(new Pose2d(3.55, 2.8, Rotation2d.fromDegrees(60))),
    CORAL_D(new Pose2d(3.95, 2.65, Rotation2d.fromDegrees(60))),
    CORAL_E(new Pose2d(5.1, 2.7, Rotation2d.fromDegrees(120))),
    CORAL_F(new Pose2d(5.4, 2.85, Rotation2d.fromDegrees(120))),
    CORAL_G(new Pose2d(6.05, 3.85, Rotation2d.fromDegrees(180))),
    CORAL_H(new Pose2d(5.928, 4.181, Rotation2d.fromDegrees(180))),
    CORAL_I(new Pose2d(5.353, 5.224, Rotation2d.fromDegrees(-120))),
    CORAL_J(new Pose2d(5.06, 5.37, Rotation2d.fromDegrees(-120))),
    CORAL_K(new Pose2d(3.92, 5.361, Rotation2d.fromDegrees(-60))),
    CORAL_L(new Pose2d(3.627, 5.166, Rotation2d.fromDegrees(-60))),
    NO_TARGET(null),
    STATION_ONE(new Pose2d(1.772, 7.343, Rotation2d.fromDegrees(128))),
    STATION_TWO(new Pose2d(1.254, 7.018, Rotation2d.fromDegrees(128))),
    STATION_THREE(new Pose2d(0.692, 6.673, Rotation2d.fromDegrees(128))),
    STATION_FOUR(new Pose2d(0.735, 1.399, Rotation2d.fromDegrees(-128))),
    STATION_FIVE(new Pose2d(1.102, 1.096, Rotation2d.fromDegrees(-128))),
    STATION_SIX(new Pose2d(1.664, 1.664, Rotation2d.fromDegrees(-128))),
    NO_STATION(null);

    private final Pose2d coralLocation;

    private AutoPathLocations(Pose2d coralLocation) {
        this.coralLocation = coralLocation;
    }

    public Pose2d getLocation() {
        if(!DriverStation.getAlliance().isEmpty()) {
            if(DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) {
                if(coralLocation != null) {
                    return FlippingUtil.flipFieldPose(coralLocation);
                }
                else {
                    return coralLocation;
                }
            }
            else {
                return coralLocation;
            }
        }
        else {
            return coralLocation;
        }
    }
}
