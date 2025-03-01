// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoTeleop;
import frc.robot.commands.Music;
import frc.robot.generated.CompTunerConstants;
import frc.robot.generated.PracticeTunerConstants;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.LEDStates;
import frc.robot.subsystems.SuperSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.ClimbAvator.ClimbAvator;
import frc.robot.subsystems.ClimbAvator.ClimbAvatorStates;
import frc.robot.subsystems.MrPibb.MrPibb;
import frc.robot.subsystems.MrPibb.MrPibbStates;
import frc.robot.subsystems.Swerve.Swerve;

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
    private final JoystickButton autoTrackButton = new JoystickButton(buttons, 1);

    /* Triggers */
    private final Trigger l1Trigger = OperatorController.getL1Trigger();
    private final Trigger l2Trigger = OperatorController.getL2Trigger();
    private final Trigger l3Trigger = OperatorController.getL3Trigger();
    private final Trigger l4Trigger = OperatorController.getL4Trigger();
    private final Trigger slowDownTrigger;
    private final Trigger climbLEDTrigger;
    private final Trigger funeralLEDTrigger;
    private final Trigger botFullAlgaeLEDTrigger;
    private final Trigger botFullCoralLEDTrigger;

    /* Subsystems */
    private final Swerve s_Swerve = createSwerve();
    private final Vision s_Vision = new Vision(s_Swerve);
    private final MrPibb s_MrPibb = new MrPibb();
    private final ClimbAvator s_ClimbAvator = new ClimbAvator();
    private final LED s_LED = new LED();
    private final SuperSubsystem s_SuperSubsystem = new SuperSubsystem(s_ClimbAvator, s_MrPibb);

    private double multiplier = 1;

    /* Commands */
    private final Music c_Music = new Music(s_Swerve, s_MrPibb, s_ClimbAvator);
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.Swerve.MAX_SPEED * Constants.Controllers.STICK_DEADBAND)
            .withRotationalDeadband(Constants.Swerve.MAX_ANGULAR_RATE * Constants.Controllers.STICK_DEADBAND) 
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.FieldCentricFacingAngle align = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(Constants.Swerve.MAX_SPEED * Constants.Controllers.STICK_DEADBAND)
            .withRotationalDeadband(Constants.Swerve.MAX_ANGULAR_RATE * Constants.Controllers.STICK_DEADBAND)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withMaxAbsRotationalRate(Constants.Swerve.MAX_ANGULAR_RATE)
            .withHeadingPID(2, 0, 0);

    /* Telemetry */
    private final Telemetry logger = new Telemetry(Constants.Swerve.MAX_SPEED);

    private final SendableChooser<Command> autoChooser;

    private void configureBindings() {

        s_Swerve.setDefaultCommand(
            s_Swerve.applyRequest(() ->
                drive.withVelocityX(translationController.getRawAxis(translationAxis) * multiplier * Constants.Swerve.MAX_SPEED)
                    .withVelocityY(translationController.getRawAxis(strafeAxis) * multiplier * Constants.Swerve.MAX_SPEED) 
                    .withRotationalRate(rotationController.getRawAxis(rotationAxis)  * Constants.Swerve.MAX_ANGULAR_RATE)
            )
        );

        autoTrackButton.whileTrue(new AutoTeleop(s_Swerve, s_SuperSubsystem).andThen(holdPosition()));

        slowDownTrigger.whileTrue(Commands.runOnce(() -> multiplier = 0.5)).whileFalse(Commands.runOnce(() -> multiplier = 1));

        translationButton.onTrue(Commands.runOnce(() -> s_Swerve.seedFieldCentric(), s_Swerve));

        xboxController.x().onTrue(s_SuperSubsystem.protectState());

        /* Coral Scoring States */
        xboxController.back().and(xboxController.a()).whileTrue(s_SuperSubsystem.l1State().alongWith(new InstantCommand(() -> OperatorController.clearLevelEntry())));
        xboxController.back().and(xboxController.b()).whileTrue(s_SuperSubsystem.l2State().alongWith(new InstantCommand(() -> OperatorController.clearLevelEntry())));
        xboxController.back().and(xboxController.x()).whileTrue(s_SuperSubsystem.l3State().alongWith(new InstantCommand(() -> OperatorController.clearLevelEntry())));
        xboxController.back().and(xboxController.y()).whileTrue(s_SuperSubsystem.l4State().alongWith(new InstantCommand(() -> OperatorController.clearLevelEntry())));

        l1Trigger.onTrue(s_SuperSubsystem.l1State().alongWith(new InstantCommand(() -> OperatorController.clearLevelEntry())));
        l2Trigger.onTrue(s_SuperSubsystem.l2State().alongWith(new InstantCommand(() -> OperatorController.clearLevelEntry())));
        l3Trigger.onTrue(s_SuperSubsystem.l3State().alongWith(new InstantCommand(() -> OperatorController.clearLevelEntry())));
        l4Trigger.onTrue(s_SuperSubsystem.l4State().alongWith(new InstantCommand(() -> OperatorController.clearLevelEntry())));

        /* Intake States */
        xboxController.y().onTrue(s_SuperSubsystem.stationState());
        xboxController.rightBumper().onTrue(s_SuperSubsystem.preStageState());
        xboxController.leftTrigger(0.3).and(xboxController.rightTrigger(0.3).negate()).whileTrue(s_MrPibb.runLoader().asProxy()).onFalse(s_MrPibb.stopLoader().asProxy().andThen(s_MrPibb.stopThumb().asProxy()));
        xboxController.leftBumper().onTrue(Commands.either(s_MrPibb.runLoaderReverseTrough(), s_MrPibb.runLoaderReverse(), () -> s_MrPibb.getState().equals(MrPibbStates.L1.name()))).onFalse(s_MrPibb.runUntilFullAlgae().asProxy());
        xboxController.rightTrigger(0.3).and(xboxController.leftTrigger(0.3).negate()).onTrue(s_MrPibb.runThumbForward().asProxy()).onFalse(s_MrPibb.stopThumb().asProxy());
        xboxController.povLeft().onTrue(s_SuperSubsystem.groundCoralState());
        xboxController.povRight().onTrue(s_SuperSubsystem.groundAlgaeState());
        new Trigger(s_MrPibb.botFullAlgae()).onFalse(Commands.runOnce(() -> s_MrPibb.stopLoader(), s_MrPibb));
        
        /* Climbing States */
        xboxController.povUp().onTrue(s_SuperSubsystem.climbState());
        xboxController.povDown().onTrue(s_SuperSubsystem.climbPullState());
        xboxController.start().onTrue(s_ClimbAvator.bilboBagginsTheBackForward()).onFalse(s_ClimbAvator.bilboBagginsTheBackStop());
        xboxController.leftTrigger(0.3).and(xboxController.rightTrigger(0.3)).onTrue(s_ClimbAvator.bilboBagginsTheBackBackward()).onFalse(s_ClimbAvator.bilboBagginsTheBackStop());
        
        /* Algae States */
        xboxController.rightStick().onTrue(s_SuperSubsystem.lowerAlgaeState());
        xboxController.leftStick().onTrue(s_SuperSubsystem.upperAlgaeState());
        xboxController.a().onTrue(s_SuperSubsystem.processorState());
        xboxController.b().onTrue(s_SuperSubsystem.bargeState());

        /* LEDs */
        climbLEDTrigger.whileTrue(Commands.runOnce(() -> s_LED.setState(LEDStates.CLIMBED), s_LED));
        funeralLEDTrigger.whileTrue(Commands.runOnce(() -> s_LED.setState(LEDStates.FUNERAL), s_LED).ignoringDisable(true));
        botFullAlgaeLEDTrigger.onTrue(Commands.runOnce(() -> s_LED.setState(LEDStates.ALGAE_FULL), s_LED));
        botFullCoralLEDTrigger.onTrue(Commands.runOnce(() -> s_LED.setState(LEDStates.CORAL_FULL), s_LED));
        botFullAlgaeLEDTrigger.and(botFullCoralLEDTrigger).and(funeralLEDTrigger).and(climbLEDTrigger).whileFalse(Commands.runOnce(() -> s_LED.setState(LEDStates.BOT_EMPTY), s_LED));

        s_Swerve.registerTelemetry(logger::telemeterize);
    }

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.setDefaultOption("DO NOTHING", Commands.none());
        autoChooser.addOption("L4 Left", new PathPlannerAuto("L4 Right", true));
        autoChooser.addOption("L4 Left CAC DS", new PathPlannerAuto("L4 Right CAC DS", true));
        Shuffleboard.getTab("Auto").add(autoChooser).withPosition(0, 0).withSize(2, 1);
        slowDownTrigger = new Trigger(() -> s_ClimbAvator.getState().equals(ClimbAvatorStates.L4) || s_ClimbAvator.getState().equals(ClimbAvatorStates.BARGE));
        climbLEDTrigger = new Trigger(() -> s_ClimbAvator.getState().equals(ClimbAvatorStates.CLIMB_PULL) && s_ClimbAvator.waitUntilShoulderSafe().isFinished());
        funeralLEDTrigger = new Trigger(s_Vision.funeral());
        botFullAlgaeLEDTrigger = new Trigger(s_SuperSubsystem.botFullAlgae());
        botFullCoralLEDTrigger = new Trigger(s_SuperSubsystem.botFullCoral());

        align.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        align.HeadingController.setTolerance(0.1);

        configureBindings();
    }

    public Command getAutonomousCommand() {
        Logger.recordOutput("Auto/Selected Auto", autoChooser.getSelected().getName());
        return autoChooser.getSelected();
    }

    public Command getInitialProtectCommand() {
        return s_SuperSubsystem.protectState();
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

    public LED getLEDSystem() {
        return s_LED;
    }

    public Command getMusicCommand() {
        return c_Music;
    }

    public Command holdPosition() {
        return s_Swerve.applyRequest(() ->
                align.withVelocityX(getSpeeds().vxMetersPerSecond * multiplier * Constants.Swerve.MAX_SPEED)
                    .withVelocityY(getSpeeds().vyMetersPerSecond * multiplier * Constants.Swerve.MAX_SPEED) 
                    .withTargetDirection(s_Swerve.getRotationTarget())
            );
    }

    public ChassisSpeeds getSpeeds() {
        if(Constants.DEBUG) {
            SmartDashboard.putNumber("Chassis Speeds X", ChassisSpeeds.fromRobotRelativeSpeeds(translationController.getRawAxis(translationAxis), s_Swerve.getYSpeed(), 0, s_Swerve.getState().Pose.getRotation()).vxMetersPerSecond);
            SmartDashboard.putNumber("Chassis Speeds Y", ChassisSpeeds.fromRobotRelativeSpeeds(translationController.getRawAxis(translationAxis), s_Swerve.getYSpeed(), 0, s_Swerve.getState().Pose.getRotation()).vyMetersPerSecond);
            SmartDashboard.putNumber("Chassis Speeds X Scaled", ChassisSpeeds.fromRobotRelativeSpeeds(translationController.getRawAxis(translationAxis), s_Swerve.getYSpeed(), 0, s_Swerve.getState().Pose.getRotation()).vxMetersPerSecond  * multiplier * Constants.Swerve.MAX_SPEED);
            SmartDashboard.putNumber("Chassis Speeds Y Scaled", ChassisSpeeds.fromRobotRelativeSpeeds(translationController.getRawAxis(translationAxis), s_Swerve.getYSpeed(), 0, s_Swerve.getState().Pose.getRotation()).vyMetersPerSecond  * multiplier * Constants.Swerve.MAX_SPEED);    
        }
        return ChassisSpeeds.fromRobotRelativeSpeeds(translationController.getRawAxis(translationAxis), s_Swerve.getYSpeed(), 0, s_Swerve.getState().Pose.getRotation());
    }

    public Swerve createSwerve() {
        Logger.recordOutput("Is Practice Bot", Constants.PRACTICE_BOT);
        return (Constants.PRACTICE_BOT) ? PracticeTunerConstants.createDrivetrain() : CompTunerConstants.createDrivetrain();
    }
}
