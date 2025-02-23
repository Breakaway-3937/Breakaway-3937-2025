package frc.robot.subsystems.Swerve;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public enum AutoPathLocations {
    //TODO: Update with new values.
    CORAL_A(new Pose2d(3.156, 4.169, Rotation2d.fromDegrees(0))),
    CORAL_B(new Pose2d(3.146, 3.852, Rotation2d.fromDegrees(0))),
    CORAL_C(new Pose2d(3.695, 2.928, Rotation2d.fromDegrees(60))),
    CORAL_D(new Pose2d(3.935, 2.774, Rotation2d.fromDegrees(60))),
    CORAL_E(new Pose2d(5.023, 2.774, Rotation2d.fromDegrees(120))),
    CORAL_F(new Pose2d(5.311, 2.938, Rotation2d.fromDegrees(120))),
    CORAL_G(new Pose2d(5.840, 3.861, Rotation2d.fromDegrees(180))),
    CORAL_H(new Pose2d(5.840, 4.198, Rotation2d.fromDegrees(180))),
    CORAL_I(new Pose2d(5.359, 5.103, Rotation2d.fromDegrees(-120))),
    CORAL_J(new Pose2d(5.051, 5.257, Rotation2d.fromDegrees(-120))),
    CORAL_K(new Pose2d(3.945, 5.257, Rotation2d.fromDegrees(-60))),
    CORAL_L(new Pose2d(3.666, 5.093, Rotation2d.fromDegrees(-60))),
    NO_TARGET(null),
    STATION_ONE(new Pose2d(1.772, 7.343, Rotation2d.fromDegrees(128))),
    STATION_TWO(new Pose2d(1.254, 7.018, Rotation2d.fromDegrees(128))),
    STATION_THREE(new Pose2d(0.692, 6.673, Rotation2d.fromDegrees(128))),
    STATION_FOUR(new Pose2d(0.735, 1.399, Rotation2d.fromDegrees(-128))),
    STATION_FIVE(new Pose2d(1.102, 1.096, Rotation2d.fromDegrees(-128))),
    STATION_SIX(new Pose2d(1.664, 1.664, Rotation2d.fromDegrees(-128))),
    NO_STATION(null);

    private final Pose2d location;

    private AutoPathLocations(Pose2d location) {
        this.location = location;
    }

    public Pose2d getLocation() {
        if(DriverStation.getAlliance().isPresent()) {
            if(DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) {
                if(location != null) {
                    return FlippingUtil.flipFieldPose(location);
                }
                else {
                    return location;
                }
            }
            else {
                return location;
            }
        }
        else {
            return location;
        }
    }
}
