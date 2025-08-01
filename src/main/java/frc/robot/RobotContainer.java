// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Music;
import frc.robot.generated.CompTunerConstants;
import frc.robot.generated.PracticeTunerConstants;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.QuestNavSubsystem;
import frc.robot.subsystems.SuperSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.ClimbAvator.ClimbAvator;
import frc.robot.subsystems.ClimbAvator.ClimbAvatorStates;
import frc.robot.subsystems.LED.LEDStates;
import frc.robot.subsystems.Soda.DrPepper;
import frc.robot.subsystems.Soda.MrPibb;
import frc.robot.subsystems.Soda.MrPibbStates;
import frc.robot.subsystems.Swerve.Swerve;
import frc.robot.subsystems.Swerve.Swerve.BranchSide;

public class RobotContainer {
    /* Controllers */
    private final Joystick translationController = new Joystick(Constants.Controllers.TRANSLATION_CONTROLLER.getPort());
    private final Joystick rotationController = new Joystick(Constants.Controllers.ROTATION_CONTROLLER.getPort());
    private final CommandXboxController xboxController = new CommandXboxController(Constants.Controllers.XBOX_CONTROLLER.getPort());
    private final Joystick buttons = new Joystick(Constants.Controllers.BUTTONS.getPort());

    /* Drive Controls */
    private final int translationAxis = Constants.Controllers.TRANSLATION_AXIS;
    private final int strafeAxis = Constants.Controllers.STRAFE_AXIS;
    private final int rotationAxis = Constants.Controllers.ROTATION_AXIS;

    /* Driver Buttons */
    private final JoystickButton translationButton = new JoystickButton(translationController, Constants.Controllers.TRANSLATION_BUTTON);
    private final JoystickButton robotCentric = new JoystickButton(buttons, 1);
    private final JoystickButton leftTrack = new JoystickButton(buttons, 7);
    private final JoystickButton rightTrack = new JoystickButton(buttons, 8);

    /* Triggers */
    private final Trigger slowDownTrigger, protectTrigger;

    /* Subsystems */
    private final Swerve s_Swerve = createSwerve();
    private final QuestNavSubsystem s_QuestNav = new QuestNavSubsystem(s_Swerve);
    private final Vision s_Vision = new Vision(s_Swerve, s_QuestNav); // Initialize Vision with Swerve and QuestNav
    private final MrPibb s_MrPibb = new MrPibb();
    private final ClimbAvator s_ClimbAvator = new ClimbAvator();
    private final DrPepper s_DrPepper = new DrPepper(isAlgae());
    private final LED s_LED = new LED();
    private final SuperSubsystem s_SuperSubsystem = new SuperSubsystem(s_ClimbAvator, s_MrPibb, s_DrPepper, s_LED, s_Vision.funeral().getAsBoolean(), () -> s_Swerve.isBackwards());
    private double multiplier = 1;

    /* Commands */
    private final Music c_Music = new Music(s_Swerve, s_MrPibb, s_DrPepper, s_ClimbAvator);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.Swerve.MAX_SPEED * Constants.Controllers.STICK_DEADBAND)
            .withRotationalDeadband(Constants.Swerve.MAX_ANGULAR_RATE * Constants.Controllers.STICK_DEADBAND) 
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.RobotCentricFacingAngle align = new SwerveRequest.RobotCentricFacingAngle()
            .withDeadband(Constants.Swerve.MAX_SPEED * Constants.Controllers.STICK_DEADBAND)
            .withRotationalDeadband(Constants.Swerve.MAX_ANGULAR_RATE * Constants.Controllers.STICK_DEADBAND)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withMaxAbsRotationalRate(Constants.Swerve.MAX_ANGULAR_RATE)
            .withHeadingPID(6.5, 0, 0);

    /* Telemetry */
    private final Telemetry logger = new Telemetry(Constants.Swerve.MAX_SPEED);

    private final SendableChooser<Command> autoChooser;

    private void configureBindings() {

        s_Swerve.setDefaultCommand(
            s_Swerve.applyRequest(() ->
                drive.withVelocityX(translationController.getRawAxis(translationAxis) * multiplier * Constants.Swerve.MAX_SPEED)
                    .withVelocityY(translationController.getRawAxis(strafeAxis) * multiplier * Constants.Swerve.MAX_SPEED) 
                    .withRotationalRate(rotationController.getRawAxis(rotationAxis) * Constants.Swerve.MAX_ANGULAR_RATE)
            )
        );

        /* Driver Buttons */
        translationButton.onTrue(Commands.runOnce(() -> s_Swerve.seedFieldCentric(), s_Swerve));
        slowDownTrigger.whileTrue(Commands.runOnce(() -> multiplier = 0.4)).whileFalse(Commands.runOnce(() -> multiplier = 1));
        robotCentric.whileTrue(robotCentricDrive());
        leftTrack.onTrue(Commands.runOnce(() -> s_LED.setState(LEDStates.BOT_ALIGNING)).alongWith(setRumble(RumbleType.kLeftRumble, 1))).whileTrue(Commands.either(s_Swerve.autoAlign(BranchSide.CENTER), s_Swerve.autoAlign(BranchSide.LEFT), isAlgae())).onFalse(Commands.runOnce(() -> s_LED.setState(LEDStates.BOT_EMPTY), s_LED).alongWith(setRumble(RumbleType.kBothRumble, 0)));
        rightTrack.onTrue(Commands.runOnce(() -> s_LED.setState(LEDStates.BOT_ALIGNING)).alongWith(setRumble(RumbleType.kRightRumble, 1))).whileTrue(Commands.either(s_Swerve.autoAlign(BranchSide.CENTER), s_Swerve.autoAlign(BranchSide.RIGHT), isAlgae())).onFalse(Commands.runOnce(() -> s_LED.setState(LEDStates.BOT_EMPTY), s_LED).alongWith(setRumble(RumbleType.kBothRumble, 0)));

        /* Weird Button States */
        xboxController.a().onTrue(Commands.either(s_SuperSubsystem.l1State(), s_SuperSubsystem.processorState(), xboxController.back()));
        xboxController.b().onTrue(Commands.either(s_SuperSubsystem.l2State(), s_SuperSubsystem.bargeState(), xboxController.back()));
        xboxController.x().onTrue(Commands.either(s_SuperSubsystem.l3State(), s_SuperSubsystem.protectState(), xboxController.back()));
        xboxController.y().onTrue(Commands.either(s_SuperSubsystem.l4State(), s_SuperSubsystem.stationState(), xboxController.back()));

        /* Intake States */
        protectTrigger.onTrue(s_SuperSubsystem.protectState());

        xboxController.rightBumper().onTrue(s_SuperSubsystem.preStageState());
        xboxController.leftTrigger(0.3).and(xboxController.rightTrigger(0.3).negate()).onTrue(s_DrPepper.runLoader().alongWith(setRumble(RumbleType.kBothRumble, 1)))
                                                                                                                                    .onFalse(s_DrPepper.stopLoader().alongWith(setRumble(RumbleType.kBothRumble, 0))
                                                                                                                                             .andThen(Commands.either(Commands.either(s_DrPepper.runLoaderReverseBarge(), s_DrPepper.runLoaderReverseTrough(), () -> !s_SuperSubsystem.getClimbAvatorState().equals(ClimbAvatorStates.PROCESSOR)).andThen(Commands.waitUntil(() -> !s_DrPepper.botFullAlgae().getAsBoolean())).andThen(s_DrPepper.stopLoader()), 
                                                                                                                                             Commands.none(), 
                                                                                                                                             () -> s_SuperSubsystem.getClimbAvatorState().equals(ClimbAvatorStates.PROCESSOR) || s_SuperSubsystem.getClimbAvatorState().equals(ClimbAvatorStates.BARGE))));
                                                                                                                                             
        xboxController.leftBumper().onTrue(Commands.either(s_DrPepper.runLoaderReverseTrough(), s_DrPepper.runLoaderReverse(), () -> s_MrPibb.getState().equals(MrPibbStates.L1.name()))).onFalse(s_DrPepper.stopThumb().andThen(s_DrPepper.stopLoader()));
        xboxController.rightTrigger(0.3).and(xboxController.leftTrigger(0.3).negate()).onTrue(Commands.runOnce(() -> s_DrPepper.removeDefaultCommand()).andThen(Commands.either(s_DrPepper.runThumbBackward(), Commands.either(s_DrPepper.runThumbForwardL2L3(), s_DrPepper.runThumbForward(), isL2L3()), () -> s_Swerve.isBackwards())).andThen(s_DrPepper.runLoaderSlowly()).alongWith(setRumble(RumbleType.kBothRumble, 1))).onFalse(Commands.runOnce(() -> s_DrPepper.setDefaultCommand(s_DrPepper.center())).andThen(s_DrPepper.stopThumb()).andThen(s_DrPepper.stopLoader()).alongWith(setRumble(RumbleType.kBothRumble, 0)));
        xboxController.povLeft().onTrue(s_SuperSubsystem.groundCoralState());
        xboxController.povRight().onTrue(s_SuperSubsystem.groundAlgaeState());
        
        /* Climbing States */
        xboxController.povUp().onTrue(s_SuperSubsystem.climbState());
        xboxController.povDown().onTrue(Commands.either(s_SuperSubsystem.climbPullState(), s_SuperSubsystem.lollipopState(), () -> s_ClimbAvator.getState().equals(ClimbAvatorStates.CLIMB)));
        xboxController.start().onTrue(s_ClimbAvator.bilboBagginsTheBackForward()).onFalse(s_ClimbAvator.bilboBagginsTheBackStop());
        xboxController.leftTrigger(0.3).and(xboxController.rightTrigger(0.3)).onTrue(s_ClimbAvator.bilboBagginsTheBackBackward()).onFalse(s_ClimbAvator.bilboBagginsTheBackStop());
        
        /* Algae States */
        xboxController.rightStick().onTrue(s_SuperSubsystem.lowerAlgaeState());
        xboxController.leftStick().onTrue(s_SuperSubsystem.upperAlgaeState());

        s_Swerve.registerTelemetry(logger::telemeterize);
    }

    public RobotContainer() {
        NamedCommands.registerCommand("ScoreCoralNoAlign", s_SuperSubsystem.scoreCoral(s_Swerve.hitReef(), s_Swerve.stop()));
        NamedCommands.registerCommand("ScoreCoral", s_SuperSubsystem.scoreCoralAlign(s_Swerve.autoReefCorrection()));
        NamedCommands.registerCommand("ScoreCoralFast", s_SuperSubsystem.scoreCoralAlignFast(s_Swerve.autoReefCorrection()));
        NamedCommands.registerCommand("ScoreCoralL1", s_SuperSubsystem.scoreCoralL1(s_Swerve.hitReef(), s_Swerve.stop()));
        NamedCommands.registerCommand("Load", s_SuperSubsystem.load());
        NamedCommands.registerCommand("Center", s_SuperSubsystem.center());
        NamedCommands.registerCommand("Condense", s_SuperSubsystem.condenseAuto());
        NamedCommands.registerCommand("TushPush", s_SuperSubsystem.tushPush(s_Swerve.hitRobot(), s_Swerve.stop()));
        NamedCommands.registerCommand("MakeCoachTHappy", getInitialPrestageCommand());
        NamedCommands.registerCommand("Pickup", s_SuperSubsystem.pickup(s_Swerve.hitReef(), s_Swerve.stop()));
        NamedCommands.registerCommand("ScoreAlgae", s_SuperSubsystem.scoreAlgae());
        NamedCommands.registerCommand("Barge", s_SuperSubsystem.bargeState());
        NamedCommands.registerCommand("L1", s_SuperSubsystem.l1State());
        NamedCommands.registerCommand("L4", s_SuperSubsystem.l4State());
        NamedCommands.registerCommand("LowerAlgae", s_SuperSubsystem.lowerAlgaeState());
        NamedCommands.registerCommand("UpperAlgae", s_SuperSubsystem.upperAlgaeState());
        NamedCommands.registerCommand("StationAlign", s_Swerve.autoReefCorrection()); 
        NamedCommands.registerCommand("LoadBriefly", s_SuperSubsystem.loadBriefly());
        autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.setDefaultOption("DO NOTHING", Commands.none());
        autoChooser.addOption("Tush Push L4 Left", new PathPlannerAuto("Tush Push L4 Right", true).withName("Tush Push L4 Left"));
        autoChooser.addOption("L4 Left", new PathPlannerAuto("L4 Right", true).withName("L4 Left"));
        autoChooser.addOption("L4 Left Fast", new PathPlannerAuto("L4 Right Fast", true).withName("L4 Left Fast"));
        autoChooser.addOption("L4 Back Left", new PathPlannerAuto("L4 Back Right", true).withName("L4 Back Left"));
        autoChooser.addOption("QuestNavAUTO", new PathPlannerAuto("QuestNavAuto", true).withName("QuestNavAuto"));
        Shuffleboard.getTab("Auto").add(autoChooser).withPosition(0, 0).withSize(2, 1);

        slowDownTrigger = new Trigger(() -> DriverStation.isTeleop() && (s_ClimbAvator.getState().equals(ClimbAvatorStates.L4) || s_ClimbAvator.getState().equals(ClimbAvatorStates.BARGE)));
        protectTrigger = new Trigger(() -> s_ClimbAvator.getState().equals(ClimbAvatorStates.GROUND_CORAL) && (s_DrPepper.botFullAlgae().getAsBoolean() || s_DrPepper.botFullCoral().getAsBoolean()));

        align.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        align.HeadingController.setTolerance(0.1);

        /*var estimatedFrontPose = s_Vision.getEstimatedFrontPose();
        if (estimatedFrontPose.isPresent() && !s_Vision.badTags(estimatedFrontPose)) {
            Pose2d initialPose = estimatedFrontPose.get().estimatedPose.toPose2d();
            s_Swerve.resetPose(initialPose);
            s_QuestNav.setQuestNavPose(initialPose);
        } else {
            System.out.println("PhotonVision did not detect valid tags for initial pose. Starting at default odometry.");
            s_QuestNav.setQuestNavPose(new Pose2d());
        }*/
        Pose2d pose = new Pose2d(1.22, 4.02, new Rotation2d());
        s_Swerve.resetPose(pose);
        s_QuestNav.setQuestNavPose(pose);

        configureBindings();
    }

    public Command getAutonomousCommand() {
        Logger.recordOutput("Auto/Selected Auto", autoChooser.getSelected().getName());
        SmartDashboard.putString("Current Auto", autoChooser.getSelected().getName());
        return autoChooser.getSelected();
    }

    public Vision getVisionSystem() {
        return s_Vision;
    }

    public Swerve getSwerveSystem() {
        return s_Swerve;
    }

    public ClimbAvator getClimbAvatorSystem() {
        return s_ClimbAvator;
    }

    public DrPepper getDrPepper() {
        return s_DrPepper;
    }

    public LED getLEDSystem() {
        return s_LED;
    }

    public Command getMusicCommand() {
        return c_Music;
    }

    public Command getInitialPrestageCommand() {
        return s_SuperSubsystem.preStageState();
    }

    public Command getInitialProtectCommand() {
        return s_SuperSubsystem.protectState();
    }

    public Command setRumble(RumbleType type, double percent) {
        return Commands.runOnce(() -> xboxController.setRumble(type, percent));
    }

    public BooleanSupplier isAlgae() {
        return () -> s_ClimbAvator.getState().equals(ClimbAvatorStates.UPPER_ALGAE) || s_ClimbAvator.getState().equals(ClimbAvatorStates.LOWER_ALGAE) || s_ClimbAvator.getState().equals(ClimbAvatorStates.GROUND_ALGAE) || s_ClimbAvator.getState().equals(ClimbAvatorStates.PROCESSOR) || s_ClimbAvator.getState().equals(ClimbAvatorStates.BARGE);
    }

    public BooleanSupplier isL2L3() {
        return () -> s_ClimbAvator.getState().equals(ClimbAvatorStates.L2) || s_ClimbAvator.getState().equals(ClimbAvatorStates.L3);
    }

    public Command robotCentricDrive() {
        return s_Swerve.applyRequest(() ->
                align.withVelocityX(translationController.getRawAxis(translationAxis) * multiplier * Constants.Swerve.MAX_SPEED)
                    .withVelocityY(translationController.getRawAxis(strafeAxis) * multiplier * Constants.Swerve.MAX_SPEED)
                    .withTargetDirection(s_Swerve.alignRot)
            );       
    }

    public Swerve createSwerve() {
        Logger.recordOutput("Is Practice Bot", Constants.PRACTICE_BOT);
        SmartDashboard.putBoolean("Is Practice Bot", Constants.PRACTICE_BOT);
        return (Constants.PRACTICE_BOT) ? PracticeTunerConstants.createDrivetrain() : CompTunerConstants.createDrivetrain();
    }

    public QuestNavSubsystem getQuestNavSubsystem() {
        return s_QuestNav;
    }
}
