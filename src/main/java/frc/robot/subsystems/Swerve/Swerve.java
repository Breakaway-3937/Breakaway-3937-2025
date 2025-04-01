package frc.robot.subsystems.Swerve;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static edu.wpi.first.math.MathUtil.isNear;
import static edu.wpi.first.units.Units.Inches;

import frc.robot.Constants;
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

    private int[] blueTags = {17, 18, 19, 20, 21, 22};
    private int[] redTags = {6, 7, 8, 9, 10, 11}; 
    private int[] currentTags = (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue)) ? blueTags : redTags;
    private Map<Pose2d, Integer> branches = new HashMap<>();

    private final PPHolonomicDriveController driveController;
    private AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    private final SwerveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    private final SwerveRequest.RobotCentric auto = new SwerveRequest.RobotCentric()
            .withVelocityX(0).withVelocityY(0);

    PathConstraints constraints = new PathConstraints(
        5.0, 5.0,
        Units.degreesToRadians(720.0), Units.degreesToRadians(720.0));

    public Swerve(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configPathplanner();
        driveController = new PPHolonomicDriveController(new PIDConstants(8, 0, 0), new PIDConstants(7, 0, 0));
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
        configPathplanner();
        driveController = new PPHolonomicDriveController(new PIDConstants(8, 0, 0), new PIDConstants(7, 0, 0));
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
        configPathplanner();
        driveController = new PPHolonomicDriveController(new PIDConstants(8, 0, 0), new PIDConstants(7, 0, 0));
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

    public Command finalAdjustment(Pose2d goTo) {
        var currentState = getState();
        PathPlannerTrajectoryState goalEndState = new PathPlannerTrajectoryState();
        goalEndState.pose = goTo;

        driveController.reset(currentState.Pose, currentState.Speeds);
        var speeds = driveController.calculateRobotRelativeSpeeds(currentState.Pose, goalEndState);

        return applyRequest(() -> pathApplyRobotSpeeds.withSpeeds(speeds));
    }

    public Command pathToReef(BranchSide side) {
        var robotState = getState();
        var goTo = closetBranch();
        
        Translation2d offset;
        switch (side) {
            case LEFT -> offset = new Translation2d(Inches.of(0), Inches.of(0));
            case RIGHT -> offset = new Translation2d(Inches.of(0), Inches.of(0));    
            case CENTER -> offset = new Translation2d(Inches.of(0), Inches.of(0));
            default -> offset = new Translation2d(0 ,0 );
        }

        var translation = goTo.getTranslation().plus(offset).rotateBy(goTo.getRotation());
        goTo = new Pose2d(translation, goTo.getRotation());

        //TODO dir of travel not done
        Rotation2d directionOfTravel = new Rotation2d(robotState.Speeds.vxMetersPerSecond, robotState.Speeds.vyMetersPerSecond); 
        Pose2d robotPosition = new Pose2d(robotState.Pose.getTranslation(), directionOfTravel);

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(robotPosition, goTo); //goTo is reef branch

        double currentSpeed = new Translation2d(robotState.Speeds.vxMetersPerSecond, robotState.Speeds.vyMetersPerSecond).getNorm();

        PathPlannerPath path = new PathPlannerPath(waypoints, constraints, new IdealStartingState(currentSpeed, directionOfTravel), new GoalEndState(0, goTo.getRotation()));
        path.preventFlipping = true;

        return AutoBuilder.followPath(path).andThen(finalAdjustment(goTo));
    }

    public Command autoAlign(BranchSide side) {
        return Commands.defer(() -> pathToReef(side), Set.of(this));
    }

    public Pose2d closetBranch() {
        return getState().Pose.nearest(new ArrayList<Pose2d>(branches.keySet()));
    }

    public void makePoseList() {
        for(int i = 0; i < currentTags.length; i++) {
            branches.put((field.getTagPose(currentTags[i]).get().toPose2d()), currentTags[i]);
        }
    }

    public Pose2d flipPose(Pose2d notFlipped) {
        //Flips from blue to red //TODO do i need this?
        double FIELD_LENGTH = 16.54; 
        return new Pose2d(new Translation2d(FIELD_LENGTH - notFlipped.getX(), notFlipped.getY()), new Rotation2d(Math.PI).minus(notFlipped.getRotation()));
    }

    public boolean isBackwards() {
        double yaw = getState().Pose.getRotation().getDegrees();
        Pose2d nearestBranch = closetBranch(); //TODO make this in perodic to keep synced?
        int offset = (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red)) ? -180 : 0;
        int tolerance = 60;

        double expectedAngle;

        switch (branches.get(nearestBranch)) {
            case 18,7 -> expectedAngle = 180 + offset;
            case 17,8 -> expectedAngle = -120 - offset;
            case 22,9 -> expectedAngle = -60 - offset;
            case 21,10 -> expectedAngle = 0 - offset;
            case 20,11 -> expectedAngle = 60 - offset;
            case 19,6 -> expectedAngle = 120 - offset;
            default -> expectedAngle = 0;
        }

        return isNear(expectedAngle, yaw, tolerance);
    }

    public enum BranchSide {
        LEFT,
        RIGHT,
        CENTER;
    }

    public Command hitReef() {
        return applyRequest(() -> auto.withVelocityX(1).withVelocityY(0));
    }

    public Command hitReefBackwards() {
        return applyRequest(() -> auto.withVelocityX(-1).withVelocityY(0));
    }

    public Command unhitReef() {
        return applyRequest(() -> auto.withVelocityX(-1).withVelocityY(0));
    }

    public Command hitRobot() {
        return applyRequest(() -> auto.withVelocityX(-1).withVelocityY(0));
    }

    public Command hitReefTeleopBackwards() {
        return Commands.deadline(new WaitCommand(0.5), hitReefBackwards());
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


        if(Constants.DEBUG) {
            SmartDashboard.putNumber("Rotation from pose", getState().Pose.getRotation().getDegrees());
            SmartDashboard.putBoolean("isBackwards", isBackwards());
        }
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
