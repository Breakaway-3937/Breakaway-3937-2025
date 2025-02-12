// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoTeleop;
import frc.robot.commands.CenterOnAprilTag;
import frc.robot.commands.Music;
import frc.robot.generated.CompTunerConstants;
import frc.robot.generated.PracticeTunerConstants;
import frc.robot.subsystems.SuperSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.ClimbAvator.ClimbAvator;
import frc.robot.subsystems.MrPibb.MrPibb;
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
    private final JoystickButton rotationButton = new JoystickButton(rotationController, Constants.Controllers.ROTATION_BUTTON);
    private final JoystickButton autoTrackButton = new JoystickButton(buttons, 1);
    private final JoystickButton alignButton = new JoystickButton(buttons, 8);

    /* Subsystems */
    private final Swerve s_Swerve = PracticeTunerConstants.createDrivetrain();//createSwerve();
    private final Vision s_Vision = new Vision(s_Swerve);
    private final MrPibb s_MrPibb = new MrPibb();
    private final ClimbAvator s_ClimbAvator = new ClimbAvator();
    private final SuperSubsystem s_SuperSubsystem = new SuperSubsystem(s_ClimbAvator, s_MrPibb);

    /* Commands */
    private final Music c_Music = new Music(s_Swerve, s_MrPibb, s_ClimbAvator);
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.Swerve.MAX_SPEED * Constants.Controllers.STICK_DEADBAND)
            .withRotationalDeadband(Constants.Swerve.MAX_ANGULAR_RATE * Constants.Controllers.STICK_DEADBAND) 
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    @SuppressWarnings("unused")
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    @SuppressWarnings("unused")
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    /* Telemetry */
    private final Telemetry logger = new Telemetry(Constants.Swerve.MAX_SPEED);

    private final SendableChooser<Command> autoChooser;

    private void configureBindings() {

        s_Swerve.setDefaultCommand(
            s_Swerve.applyRequest(() ->
                drive.withVelocityX(translationController.getRawAxis(translationAxis) * Constants.Swerve.MAX_SPEED)
                    .withVelocityY(translationController.getRawAxis(strafeAxis) * Constants.Swerve.MAX_SPEED) 
                    .withRotationalRate(rotationController.getRawAxis(rotationAxis)  * Constants.Swerve.MAX_ANGULAR_RATE)
            )
        );

        translationButton.onTrue(Commands.runOnce(() -> s_Swerve.seedFieldCentric(), s_Swerve));

        //Coral Scoring States
        xboxController.povDown().onTrue(s_SuperSubsystem.level1State().andThen(s_SuperSubsystem.runSubsystems()));
        xboxController.povUp().onTrue(s_SuperSubsystem.level2State().andThen(s_SuperSubsystem.runSubsystems()));
        xboxController.a().onTrue(s_SuperSubsystem.level3State().andThen(s_SuperSubsystem.runSubsystems()));
        xboxController.y().onTrue(s_SuperSubsystem.level4State().andThen(s_SuperSubsystem.runSubsystems()));

        //Intake States
        xboxController.leftTrigger(0.3).onTrue(s_SuperSubsystem.loadState()); // TODO Test me
        xboxController.rightBumper().onTrue(s_SuperSubsystem.preStageState().andThen(s_SuperSubsystem.runSubsystems()));
        xboxController.start().onTrue(s_MrPibb.runLoader()).onFalse(s_MrPibb.stopLoader());
        xboxController.back().onTrue(s_MrPibb.runThumbForward()).onFalse(s_MrPibb.stopThumb());
 
        //Climbing States
        //xboxController.leftStick().onTrue(s_SuperSubsystem.climbState().alongWith(s_SuperSubsystem.runSubsystems()));
        //xboxController.rightStick().onTrue(s_SuperSubsystem.zeroState().alongWith(s_SuperSubsystem.runSubsystems()));
        
        //Algae States
        xboxController.povRight().onTrue(s_SuperSubsystem.lowerAlgaeState().alongWith(s_SuperSubsystem.runSubsystems()));
        xboxController.b().onTrue(s_SuperSubsystem.upperAlgaeState().alongWith(s_SuperSubsystem.runSubsystems()));

        autoTrackButton.whileTrue(new AutoTeleop(s_Swerve, s_Vision, s_SuperSubsystem));
        alignButton.or(rotationButton).whileTrue(new CenterOnAprilTag(s_Swerve, s_Vision, 0));

        s_Swerve.registerTelemetry(logger::telemeterize);
    }

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.setDefaultOption("DO NOTHING", Commands.none());
        autoChooser.addOption("L4 Left", new PathPlannerAuto("L4 Right", true));
        autoChooser.addOption("L4 Left CAC DS", new PathPlannerAuto("L4 Right CAC DS", true));
        Shuffleboard.getTab("Auto").add(autoChooser).withPosition(0, 0).withSize(2, 1);
        configureBindings();
    }

    public Command getAutonomousCommand() {
        Logger.recordOutput("Selected Auto", autoChooser.getSelected().getName());
        return autoChooser.getSelected();
    }

    //TODO
    public Command getInitialProtectCommand() {
        return s_SuperSubsystem.protectState().alongWith(s_SuperSubsystem.runSubsystems());
    }

    public Vision getVisionSystem() {
        return s_Vision;
    }

    public Swerve getSwerveSystem() {
        return s_Swerve;
    }

    public Command getMusicCommand() {
        return c_Music;
    }

    public Swerve createSwerve() {
        return (Constants.PRACTICE_BOT) ? PracticeTunerConstants.createDrivetrain() : CompTunerConstants.createDrivetrain();
    }
}

//FIXME: Run SysId with robot.
/*
// Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
*/
