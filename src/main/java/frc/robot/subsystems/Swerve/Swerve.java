package frc.robot.subsystems.Swerve;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static edu.wpi.first.wpilibj2.command.Commands.either;

import frc.robot.generated.PracticeTunerConstants.TunerSwerveDrivetrain;

public class Swerve extends TunerSwerveDrivetrain implements Subsystem {
    private static final double simLoopPeriod = 0.005; // 5 ms
    private Notifier simNotifier = null;
    private double lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d blueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d redAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false; 

    private ArrayList<Pose2d> poseList;
    private List<AutoPathLocations> leftTargets = Arrays.asList(AutoPathLocations.CORAL_A, AutoPathLocations.CORAL_C, AutoPathLocations.CORAL_E, AutoPathLocations.CORAL_G, AutoPathLocations.CORAL_I, AutoPathLocations.CORAL_K); 
    private List<AutoPathLocations> rightTargets = Arrays.asList(AutoPathLocations.CORAL_B, AutoPathLocations.CORAL_D, AutoPathLocations.CORAL_F, AutoPathLocations.CORAL_H, AutoPathLocations.CORAL_J, AutoPathLocations.CORAL_L); 

    private final SwerveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    private final SwerveRequest.RobotCentric auto = new SwerveRequest.RobotCentric()
            .withVelocityX(0).withVelocityY(0);

    PathConstraints constraints = new PathConstraints(
        4.0, 4.0,
        Units.degreesToRadians(720.0), Units.degreesToRadians(720.0));

    public Swerve(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        //configPathplanner();
        this.setStateStdDevs(VecBuilder.fill(0.05, 0.05, 0.05));
    }

    public Swerve(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        //configPathplanner();
        this.setStateStdDevs(VecBuilder.fill(0.05, 0.05, 0.05));
    }

    public Swerve(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        //configPathplanner();
        this.setStateStdDevs(VecBuilder.fill(0.05, 0.05, 0.05));
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public void configPathplanner() {
        RobotConfig config;
        try {

            config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                () -> getState().Pose, 
                this::resetPose,    
                () -> getState().Speeds, 
                (speeds, feedforwards) -> setControl(
                    pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    new PIDConstants(8, 0, 0),
                    new PIDConstants(7, 0, 0)
                ),
                config,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this
            );
        }
        catch(IOException | ParseException e) {
            e.printStackTrace();
        }

        Pathfinding.setPathfinder(new LocalADStar());
    }

    public Rotation2d getRotationTarget() {
        var target = findNearestTarget(false);
        if(target.get() != null && !target.get().getAllPathPoints().isEmpty()) {
            return target.get().getAllPathPoints().get(target.get().getAllPathPoints().size() - 1).rotationTarget.rotation();
        }
        else {
            return getState().Pose.getRotation();
        }
    }

    public Supplier<PathPlannerPath> findNearestTarget(boolean right) {
        makePoseList();
        var near = getState().Pose.nearest(poseList);
        var target = lookUpPath(near);
        AutoPathLocations goTo;

        if(target == null) {
            return null;
        }

        if(right) {
            if(leftTargets.contains(target)) {
                goTo = rightTargets.get(leftTargets.indexOf(target));
            }
            else {
                goTo = target;
            }
        }
        else {
            if(rightTargets.contains(target)) {
                goTo = leftTargets.get(rightTargets.indexOf(target));
            }
            else {
                goTo = target;
            }
        }

        Logger.recordOutput("Auto Path Location", goTo.name());

        return () -> goTo.getPath();
    }

    public void makePoseList() {
        poseList = new ArrayList<>();
        var locationList = Arrays.asList(AutoPathLocations.values());
        PathPlannerPath path;

        for(int i = 0; i < locationList.size(); i++) {
            if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                path = locationList.get(i).getPath().flipPath();
            }
            else {
                path = locationList.get(i).getPath();
            }
            
            var point = path.getAllPathPoints().get(0).position;
            var rotation = path.getIdealStartingState().rotation();
            poseList.add(new Pose2d(point, rotation));
        }
    }

    public AutoPathLocations lookUpPath(Pose2d nearest) {
        var locationList = Arrays.asList(AutoPathLocations.values());
        PathPlannerPath path;
        for(int i = 0; i < locationList.size(); i++) {
            if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                path = locationList.get(i).getPath().flipPath();
            }
            else {
                path = locationList.get(i).getPath();
            }

            if(nearest.getTranslation().equals(path.getAllPathPoints().get(0).position)) {
                return locationList.get(i);
            }
        }
        return null;
    }

    public Command pathFindToClosest(boolean right) {
        return defer(() -> 
            {
                Logger.recordOutput("Swerve/Algae Align", false);
                var target = findNearestTarget(right);
                Logger.recordOutput("Final Auto Align Path", target.get().name);
                return either(AutoBuilder.pathfindThenFollowPath(target.get(), constraints), Commands.none(), () -> target != null);
            });
    }

    public Command pathFindAndFollowToAlgae() {
        return defer(() -> {
            Logger.recordOutput("Swerve/Algae Align", true);
            var target = findNearestTarget(false);
            if(target.get() != null) {
                PathPlannerPath location = target.get();

                switch (target.get().name) {
                    case "A", "B":
                        location = AlgaeAutoPathLocations.ALGAE_AB.getPath();
                        break;
                    case "C", "D":
                        location = AlgaeAutoPathLocations.ALGAE_CD.getPath();
                        break;
                    case "E", "F":
                        location = AlgaeAutoPathLocations.ALGAE_EF.getPath();
                        break;
                    case "G", "H":
                        location = AlgaeAutoPathLocations.ALGAE_GH.getPath();
                        break;
                    case "I", "J":
                        location = AlgaeAutoPathLocations.ALGAE_IJ.getPath();
                        break;
                    case "K", "L":
                        location = AlgaeAutoPathLocations.ALGAE_KL.getPath();
                        break;
                    default:
                        location = target.get();
                        break;
                }

                PathPlannerPath finalLocation = location;
                Logger.recordOutput("Final Auto Align Path", finalLocation.name);
                return defer(() -> AutoBuilder.pathfindThenFollowPath(finalLocation, constraints));
            }
            else {
                return Commands.none();
            }
        });
    }

    public Command hitReef() {
        return applyRequest(() -> auto.withVelocityX(1).withVelocityY(0));
    }

    public Command unhitReef() {
        return applyRequest(() -> auto.withVelocityX(-1).withVelocityY(0));
    }

    public Command stop() {
        return applyRequest(() -> auto.withVelocityX(0).withVelocityY(0));
    }

    @Override
    public void periodic() {
        /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? redAlliancePerspectiveRotation
                                : blueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }

        Logger.recordOutput("Rotation Target", getRotationTarget());
    }

    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        simNotifier.startPeriodic(simLoopPeriod);
    }
}
