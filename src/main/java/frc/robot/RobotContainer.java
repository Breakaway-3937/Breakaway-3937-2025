// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
import frc.robot.subsystems.Soda.DrPepper;
import frc.robot.subsystems.Soda.MrPibb;
import frc.robot.subsystems.Soda.MrPibbStates;
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
    private final Trigger slowDownTrigger;
    private final Trigger climbLEDTrigger;
    private final Trigger funeralLEDTrigger;
    private final Trigger botFullAlgaeLEDTrigger;
    private final Trigger botFullCoralLEDTrigger;

    /* Subsystems */
    private final Swerve s_Swerve = createSwerve();
    private final Vision s_Vision = new Vision(s_Swerve);
    private final MrPibb s_MrPibb = new MrPibb();
    private final DrPepper s_DrPepper = new DrPepper();
    private final ClimbAvator s_ClimbAvator = new ClimbAvator();
    private final LED s_LED = new LED();
    private final SuperSubsystem s_SuperSubsystem = new SuperSubsystem(s_ClimbAvator, s_MrPibb, s_DrPepper);

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
            .withHeadingPID(4.5, 0, 0);

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

        slowDownTrigger.whileTrue(Commands.runOnce(() -> multiplier = 0.4)).whileFalse(Commands.runOnce(() -> multiplier = 1));

        translationButton.onTrue(Commands.runOnce(() -> s_Swerve.seedFieldCentric(), s_Swerve));

        xboxController.a().onTrue(Commands.either(s_SuperSubsystem.processorState(), s_SuperSubsystem.l1State().alongWith(new InstantCommand(() -> OperatorController.clearLevelEntry())), xboxController.back()));
        xboxController.b().onTrue(Commands.either(s_SuperSubsystem.bargeState(), s_SuperSubsystem.l2State().alongWith(new InstantCommand(() -> OperatorController.clearLevelEntry())), xboxController.back()));
        xboxController.x().onTrue(Commands.either(s_SuperSubsystem.protectState(), s_SuperSubsystem.l3State().alongWith(new InstantCommand(() -> OperatorController.clearLevelEntry())), xboxController.back()));
        xboxController.y().onTrue(Commands.either(s_SuperSubsystem.stationState(), s_SuperSubsystem.l4State().alongWith(new InstantCommand(() -> OperatorController.clearLevelEntry())), xboxController.back()));

        /* Intake States */
        xboxController.rightBumper().onTrue(s_SuperSubsystem.preStageState());
        xboxController.leftTrigger(0.3).and(xboxController.rightTrigger(0.3).negate()).whileTrue(s_DrPepper.runUntilFullCoral()).onFalse(s_DrPepper.stopLoader().andThen(s_DrPepper.stopThumb()));
        xboxController.leftBumper().onTrue(Commands.either(s_DrPepper.runLoaderReverseTrough(), s_DrPepper.runLoaderReverse(), () -> s_MrPibb.getState().equals(MrPibbStates.L1.name()))).onFalse(s_DrPepper.runUntilFullAlgae());
        xboxController.rightTrigger(0.3).and(xboxController.leftTrigger(0.3).negate()).onTrue(s_DrPepper.runThumbForward()).onFalse(s_DrPepper.stopThumb());
        xboxController.povLeft().onTrue(s_SuperSubsystem.groundCoralState());
        xboxController.povRight().onTrue(s_SuperSubsystem.groundAlgaeState());
        
        /* Climbing States */
        xboxController.povUp().onTrue(s_SuperSubsystem.climbState());
        xboxController.povDown().onTrue(s_SuperSubsystem.climbPullState());
        xboxController.start().onTrue(s_ClimbAvator.bilboBagginsTheBackForward()).onFalse(s_ClimbAvator.bilboBagginsTheBackStop());
        xboxController.leftTrigger(0.3).and(xboxController.rightTrigger(0.3)).onTrue(s_ClimbAvator.bilboBagginsTheBackBackward()).onFalse(s_ClimbAvator.bilboBagginsTheBackStop());
        
        /* Algae States */
        xboxController.rightStick().onTrue(s_SuperSubsystem.lowerAlgaeState());
        xboxController.leftStick().onTrue(s_SuperSubsystem.upperAlgaeState());

        /* LEDs */
        climbLEDTrigger.whileTrue(Commands.runOnce(() -> s_LED.setState(LEDStates.CLIMBED), s_LED));
        funeralLEDTrigger.whileTrue(Commands.runOnce(() -> s_LED.setState(LEDStates.FUNERAL), s_LED).ignoringDisable(true));
        botFullAlgaeLEDTrigger.onTrue(Commands.runOnce(() -> s_LED.setState(LEDStates.ALGAE_FULL), s_LED)).onFalse(Commands.runOnce(() -> s_DrPepper.stopLoader(), s_DrPepper));
        botFullCoralLEDTrigger.onTrue(Commands.runOnce(() -> s_LED.setState(LEDStates.CORAL_FULL), s_LED));
        botFullAlgaeLEDTrigger.and(botFullCoralLEDTrigger).and(funeralLEDTrigger).and(climbLEDTrigger).whileFalse(Commands.runOnce(() -> s_LED.setState(LEDStates.BOT_EMPTY), s_LED));

        s_Swerve.registerTelemetry(logger::telemeterize);
    }

    public RobotContainer() {
        NamedCommands.registerCommand("ScoreCoral", s_SuperSubsystem.scoreCoral(s_Swerve.hitReef(), s_Swerve.unhitReef(), s_Swerve.stop(), s_Swerve.stop()));
        NamedCommands.registerCommand("Load", s_SuperSubsystem.load());
        NamedCommands.registerCommand("Condense", s_SuperSubsystem.condenseAuto());
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

    public Command getInitialPrestageCommand() {
        return s_SuperSubsystem.preStageState();
    }

    public Command getInitialProtectCommand() {
        return s_SuperSubsystem.protectState();
    }

    public Command holdPosition() {
        return s_Swerve.applyRequest(() ->
                align.withVelocityX(translationController.getRawAxis(translationAxis) * multiplier * Constants.Swerve.MAX_SPEED)
                    .withVelocityY(0) 
                    .withTargetDirection(s_Swerve.getRotationTarget())
            );
    }

    public Swerve createSwerve() {
        Logger.recordOutput("Is Practice Bot", Constants.PRACTICE_BOT);
        return (Constants.PRACTICE_BOT) ? PracticeTunerConstants.createDrivetrain() : CompTunerConstants.createDrivetrain();
    }
}
