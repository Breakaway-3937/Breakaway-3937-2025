package frc.robot.subsystems.Swerve;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
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
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static edu.wpi.first.math.MathUtil.isNear;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Feet;

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
    private Alliance pastColor = DriverStation.getAlliance().orElse(Alliance.Blue);
    public Rotation2d alignRot = new Rotation2d();

    public int cyclesInAuto = 1;
    private StringSubscriber autoSub = NetworkTableInstance.getDefault().getTable("SmartDashboard").getStringTopic("Current Auto").subscribe("");

    private final PPHolonomicDriveController driveController;

    private AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    private final SwerveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    private final SwerveRequest.RobotCentric auto = new SwerveRequest.RobotCentric()
            .withVelocityX(0).withVelocityY(0);

    PathConstraints constraints = new PathConstraints(
        2.5, 2.5,
        Units.degreesToRadians(720.0), Units.degreesToRadians(720.0));

    private final SwerveRequest.RobotCentricFacingAngle align = new SwerveRequest.RobotCentricFacingAngle()
        .withDeadband(Constants.Swerve.MAX_SPEED * Constants.Controllers.STICK_DEADBAND)
        .withRotationalDeadband(Constants.Swerve.MAX_ANGULAR_RATE * Constants.Controllers.STICK_DEADBAND)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withMaxAbsRotationalRate(Constants.Swerve.MAX_ANGULAR_RATE)
        .withHeadingPID(4, 0, 0);

    public Swerve(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        makePoseList();
        configPathplanner();
        alignRot = getState().Pose.getRotation();
        driveController = new PPHolonomicDriveController(new PIDConstants(8, 0, 0), new PIDConstants(7, 0, 0));
        this.setStateStdDevs(VecBuilder.fill(0.05, 0.05, 0.05));
        align.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        align.HeadingController.setTolerance(0.01);
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
        PathPlannerTrajectoryState goalEndState = new PathPlannerTrajectoryState();
        goalEndState.pose = goTo;

        driveController.reset(goTo, getState().Speeds);

        return applyRequest(() -> pathApplyRobotSpeeds.withSpeeds(driveController.calculateRobotRelativeSpeeds(getState().Pose, goalEndState)));
    }

    public Command pathToReef(BranchSide side) {
        var robotState = getState();
        var goTo = closetBranch();
        List<Waypoint> waypoints;
        goTo = new Pose2d(goTo.getTranslation(), goTo.getRotation().rotateBy(Rotation2d.k180deg));
        
        Translation2d offset;
        switch (side) {
            //x is left/right,y is forward/backwards
            case LEFT -> offset = new Translation2d(Centimeters.of(17), Feet.of(-3));
            case RIGHT -> offset = new Translation2d(Centimeters.of(-17), Feet.of(-3));
            case CENTER -> offset = new Translation2d(Centimeters.of(0), Centimeters.of(-63));
            default -> offset = new Translation2d(Centimeters.of(0), Feet.of(-3));
        }

        var translation = goTo.getTranslation().plus(new Translation2d(offset.getY(), offset.getX()).rotateBy(goTo.getRotation()));
        goTo = new Pose2d(translation.getX(), translation.getY(), goTo.getRotation());

        Rotation2d directionOfTravel = new Rotation2d(robotState.Speeds.vxMetersPerSecond, robotState.Speeds.vyMetersPerSecond); 
        Pose2d robotPosition = new Pose2d(robotState.Pose.getTranslation(), directionOfTravel);

        if(isBackwards()) {
            robotPosition = new Pose2d(robotState.Pose.getTranslation(), directionOfTravel.rotateBy(Rotation2d.k180deg));
            goTo = new Pose2d(goTo.getTranslation(), goTo.getRotation().rotateBy(Rotation2d.k180deg));
            waypoints = PathPlannerPath.waypointsFromPoses(robotPosition, goTo); //goTo is reef branch
        }
        else {
            waypoints = PathPlannerPath.waypointsFromPoses(robotPosition, goTo); //goTo is reef branch
        }
        
        double currentSpeed = new Translation2d(robotState.Speeds.vxMetersPerSecond, robotState.Speeds.vyMetersPerSecond).getNorm();

        PathPlannerPath path = new PathPlannerPath(waypoints, constraints, new IdealStartingState(currentSpeed, directionOfTravel), new GoalEndState(0, goTo.getRotation()));
        path.preventFlipping = true;

        alignRot = goTo.getRotation();
        var finalTranslation = goTo.getTranslation().plus(new Translation2d(Centimeters.of(-50), offset.getMeasureX()).rotateBy(goTo.getRotation()));
        var finalMovement = new Pose2d(finalTranslation, goTo.getRotation());
        return AutoBuilder.followPath(path).andThen(finalAdjustment(finalMovement));
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

        if(Constants.DEBUG) {
            SmartDashboard.putNumberArray("Current Tags", branches.values().stream().mapToDouble(Integer::doubleValue).toArray());
        }
    }

    public boolean isBackwards() {
        double yaw = getState().Pose.getRotation().getDegrees();
        Pose2d nearestBranch = closetBranch();
        int offset = (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red)) ? -180 : 0;
        double tolerance = 60;

        double expectedAngle;

        switch (branches.get(nearestBranch)) {
            case 18,7 -> expectedAngle = 180 + offset;
            case 17,8 -> expectedAngle = -120 - offset;
            case 22,9 -> expectedAngle = -60 - offset;
            case 21,10 -> expectedAngle = 0 - offset;
            case 20,11 -> expectedAngle = 60 + offset;
            case 19,6 -> expectedAngle = 120 + offset;
            default -> expectedAngle = 0;
        }

        if(expectedAngle == 180 + offset || expectedAngle == 0 - offset) {
            yaw = Math.abs(yaw);
        }

        return isNear(expectedAngle, yaw, tolerance);
    }

    public enum BranchSide {
        LEFT,
        RIGHT,
        CENTER;
    }

    private Command reefAlignAuto() {
        int[] pointsToPull;
        Pose2d goTo;
        boolean rightSideAuto = false;
        Translation2d offset;
        Translation2d leftOffset = new Translation2d(Centimeters.of(17), Centimeters.of(-50));
        Translation2d rightOffset = new Translation2d(Centimeters.of(-17), Centimeters.of(-50));

        //blue left path: right, left, right
        //blue right path: left, right, left

        String currentAuto = autoSub.get();

        SmartDashboard.putString("Current Auto in method", currentAuto);

        switch (currentAuto) {
            case "L4 Right","Tush Push L4 Right": pointsToPull = new int[] {22, 17}; rightSideAuto = true; break; //Blue Right side: left right, left
            case "L4 Left","Tush Push L4 Left": pointsToPull = new int[] {20, 19}; rightSideAuto = false; break;
            case "L4 Back": pointsToPull = new int[] {21, 21}; rightSideAuto = true; break;
            case "L4 Back Left": pointsToPull = new int[] {21, 21}; rightSideAuto = false; break;
            default: pointsToPull = new int[] {22, 17};
        }

        if(Constants.DEBUG) {
            SmartDashboard.putBoolean("rightSideAuto?", rightSideAuto);
            SmartDashboard.putNumberArray("Points to pull",  Arrays.stream(pointsToPull).asDoubleStream().toArray());    
        }

        DriverStation.getAlliance().ifPresent((allianceColor) -> {
            if(allianceColor.equals(Alliance.Red)) { 
              for(int i = 0; i < pointsToPull.length; i++) {
                if(pointsToPull[i] == 22) {
                    pointsToPull[i] = 9; //Convert right blue to right red
                }
                if(pointsToPull[i] == 17) {
                    pointsToPull[i] = 8; //Convert right blue to right red
                }
                if(pointsToPull[i] == 20) {
                    pointsToPull[i] = 11; 
                }
                if(pointsToPull[i] == 19) {
                    pointsToPull[i] = 6;
                }
                if(pointsToPull[i] == 21) {
                    pointsToPull[i] = 10;
                }
            }}
        });

        try {
            if(cyclesInAuto == 1) {
                goTo = field.getTagPose(pointsToPull[cyclesInAuto-1]).get().toPose2d();
            }
            else if(cyclesInAuto > 1) {
                goTo = field.getTagPose(pointsToPull[1]).get().toPose2d();
            }
            else {
                goTo = field.getTagPose(pointsToPull[cyclesInAuto-1]).get().toPose2d();
            }
        }
        catch(Exception e) {
            Logger.recordOutput("Go To Auto Align", e.toString());
            Logger.recordOutput("Skipped Auto Align Step Number", cyclesInAuto);
            return Commands.none();
        }

        //For blue right side nothing done is to bool it shoudl be set up so it does the order for a blue rightside auto.
        //For left side it is configured to do the opisite of right, instead of left it does right.
        //this is not true: For red do the opsite but flipped? red leftside is blue rightside? red rightside is blue rightside?

        //Red leftside: right, left, right
        //Red rightside: left, right, left

        switch (cyclesInAuto) {
            case 1 -> offset = (rightSideAuto) ?  leftOffset : rightOffset;
            case 2 -> offset = (rightSideAuto) ?  rightOffset : leftOffset;
            case 3 -> offset = (rightSideAuto) ?  leftOffset : rightOffset;
            default -> offset = new Translation2d();
        }

        if(Constants.DEBUG) {
            SmartDashboard.putNumberArray("Non moved goto", new double[] {goTo.getX(), goTo.getY(), goTo.getRotation().getDegrees()});
        }

        goTo = new Pose2d(goTo.getTranslation(), goTo.getRotation().rotateBy(Rotation2d.k180deg));

        var translation = goTo.getTranslation().plus(new Translation2d(offset.getY(), offset.getX()).rotateBy(goTo.getRotation()));
        goTo = new Pose2d(translation, goTo.getRotation());

        if(Constants.DEBUG) {
            SmartDashboard.putNumberArray("Final Go To", new double[] {goTo.getX(), goTo.getY(), goTo.getRotation().getDegrees()});
        }

        if(getState().Pose.getX() < 2.7 || getState().Pose.getX() > 15.3) {
            cyclesInAuto--;
            var allianceColor = DriverStation.getAlliance();
            if(allianceColor.isPresent()) {
                if(allianceColor.get().equals(Alliance.Blue)) {
                    if(rightSideAuto) {
                        goTo = new Pose2d(1.551, 0.609, Rotation2d.fromDegrees(45.975));
                    }
                    else {
                        goTo = new Pose2d(1.530, 7.454, Rotation2d.fromDegrees(-54));
                    }
                }
                else {
                    if(rightSideAuto) { //This is actually left
                        goTo = new Pose2d(16.014, 7.453, Rotation2d.fromDegrees(-125));
                    }
                    else {
                        goTo = new Pose2d(16.014, 0.590, Rotation2d.fromDegrees(125));
                    }
                }
            }

        } 

        PathPlannerTrajectoryState branch = new PathPlannerTrajectoryState();
        branch.pose = goTo;
        
        cyclesInAuto++;

        return applyRequest(() -> pathApplyRobotSpeeds.withSpeeds(driveController.calculateRobotRelativeSpeeds(getState().Pose, branch)));
    }

    public Command autoReefCorrection() {
        var time = jack();
        return Commands.defer(() -> reefAlignAuto(), Set.of(this)).alongWith(time).until(() -> wheelSpeeds() < 0.05 && time.isFinished());
    }

    public Command jack() {
        return Commands.waitSeconds(0.16);
    }

    public double wheelSpeeds() {
        var robotState = getState();
        return new Translation2d(robotState.Speeds.vxMetersPerSecond, robotState.Speeds.vyMetersPerSecond).getNorm();
    }

    public Command hitReefTeleop() {
        return applyRequest(() -> align.withVelocityX(1).withTargetDirection(alignRot));
    }

    public Command hitReef() {
        return applyRequest(() -> auto.withVelocityX(1).withVelocityY(0)).withName("Hit Reef");
    }

    public Command hitRobot() {
        return applyRequest(() -> auto.withVelocityX(-1).withVelocityY(0));
    }

    public Command hitStation() {
        return applyRequest(() -> auto.withVelocityX(-1).withVelocityY(0));
    }

    public Command stop() {
        return applyRequest(() -> auto.withVelocityX(0).withVelocityY(0));
    }
    
    public boolean isAtBarge() {
        var allianceColor = DriverStation.getAlliance();
        var pose = getState().Pose;
        boolean atBarge = false;
        double blueBargeX = 0, redBargeX = 0; //TODO get x cord

        if(allianceColor.isPresent()) {
            if(allianceColor.get().equals(Alliance.Red)) {
                if(isNear(redBargeX, pose.getX(), 0.2) && pose.getY() < 4) { //(expected Y pose, current Y)
                    atBarge = true;
                }
            }
            else {
                if(isNear(blueBargeX, pose.getX(), 0.2) && pose.getY() > 4) { //(expected Y pose, current Y)
                    atBarge = true;
                }
            }
        }
        else {
            atBarge = false;
        }

        return atBarge;
    }

    public BooleanSupplier isBargeBackwards() {
        return () -> isNear(180, getState().Pose.getRotation().getDegrees(), 60); //TODO check red
    }

    @Override
    public void periodic() {
        /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing */
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? redAlliancePerspectiveRotation
                                : blueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }

        if(DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                if(!allianceColor.equals(pastColor)) {
                    currentTags = (allianceColor.equals(Alliance.Blue)) ? blueTags : redTags;
                    branches.clear();
                    makePoseList();
                    pastColor = allianceColor;
                }
            });
        }

        var command = getCurrentCommand();
        if(command != null) {
          Logger.recordOutput("Swerve/Current Command", command.getName());
        }

        if(Constants.DEBUG) {
            SmartDashboard.putNumber("Rotation from Pose", getState().Pose.getRotation().getDegrees());
            SmartDashboard.putString("Pose of Target", closetBranch().toString());
            SmartDashboard.putNumber("Branch Tag ID", branches.get(closetBranch()));
            SmartDashboard.putBoolean("Is Backwards", isBackwards());
            SmartDashboard.putString("Alliance Color", DriverStation.getAlliance().orElse(Alliance.Blue).toString());
            SmartDashboard.putString("Past Color", pastColor.toString());
            SmartDashboard.putString("Current Auto Debug", autoSub.get());
            SmartDashboard.putBoolean("At Barge", isAtBarge());
            SmartDashboard.putBoolean("Is Barge Backwards", isBargeBackwards().getAsBoolean());
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
