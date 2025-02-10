// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoTeleop;
import frc.robot.commands.CenterOnAprilTag;
import frc.robot.commands.MovementCommand;
import frc.robot.commands.Music;
import frc.robot.generated.CompTunerConstants;
import frc.robot.generated.PracticeTunerConstants;
import frc.robot.subsystems.SuperSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.ClimbAvator.ClimbAvator;
import frc.robot.subsystems.ClimbAvator.ClimbAvatorStates;
import frc.robot.subsystems.MrPibb.MrPibb;
import frc.robot.subsystems.Swerve.Swerve;

public class RobotContainer {
    //TODO: Add the correct max speed and max angular rate.
    private final double maxSpeed = PracticeTunerConstants.kSpeedAt12Volts.in(MetersPerSecond); 
    private final double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final Joystick translationController = new Joystick(Constants.Controllers.TRANSLATION_CONTROLLER.getPort());
    private final Joystick rotationController = new Joystick(Constants.Controllers.ROTATION_CONTROLLER.getPort());
    private final CommandXboxController xboxController = new CommandXboxController(Constants.Controllers.XBOX_CONTROLLER.getPort());
    private final Joystick buttons = new Joystick(Constants.Controllers.BUTTONS.getPort());

    private final JoystickButton translationButton = new JoystickButton(translationController, Constants.Controllers.TRANSLATION_BUTTON);
    private final JoystickButton rotationButton = new JoystickButton(rotationController, Constants.Controllers.ROTATION_BUTTON);
    private final JoystickButton autoTrackButton = new JoystickButton(buttons, 1);
    private final JoystickButton alignButton = new JoystickButton(buttons, 8);

    private final int translationAxis = Constants.Controllers.TRANSLATION_AXIS;
    private final int strafeAxis = Constants.Controllers.STRAFE_AXIS;
    private final int rotationAxis = Constants.Controllers.ROTATION_AXIS;

    private final Telemetry logger = new Telemetry(maxSpeed);
    private final SendableChooser<Command> autoChooser;

    private final Swerve s_Swerve = createSwerve();
    private final Vision s_Vision = new Vision(s_Swerve);
    private final MrPibb s_MrPibb = new MrPibb();
    private final ClimbAvator s_ClimbAvator = new ClimbAvator();
   private final SuperSubsystem s_SuperSubsystem = new SuperSubsystem(s_ClimbAvator, s_MrPibb);

    private final Music c_Music = new Music(s_Swerve);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * Constants.Controllers.STICK_DEADBAND)
            .withRotationalDeadband(maxAngularRate * Constants.Controllers.STICK_DEADBAND) 
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    @SuppressWarnings("unused")
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    @SuppressWarnings("unused")
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private void configureBindings() {

        s_Swerve.setDefaultCommand(
            s_Swerve.applyRequest(() ->
                drive.withVelocityX(translationController.getRawAxis(translationAxis) * maxSpeed)
                    .withVelocityY(translationController.getRawAxis(strafeAxis) * maxSpeed) 
                    .withRotationalRate(rotationController.getRawAxis(rotationAxis)  * maxAngularRate)
            )
        );

        translationButton.onTrue(Commands.runOnce(() -> s_Swerve.seedFieldCentric(), s_Swerve));

        xboxController.a().onTrue(Commands.runOnce(() -> s_SuperSubsystem.loadState()).alongWith(s_SuperSubsystem.runSubsystems()));
        xboxController.b().onTrue(Commands.runOnce(() -> s_SuperSubsystem.preStageState()).alongWith(s_SuperSubsystem.runSubsystems()));
        xboxController.x().onTrue(Commands.runOnce(() -> s_SuperSubsystem.level1State()).alongWith(s_SuperSubsystem.runSubsystems()));
        xboxController.y().onTrue(Commands.runOnce(() -> s_SuperSubsystem.level2State()).alongWith(s_SuperSubsystem.runSubsystems()));
        xboxController.start().onTrue(Commands.runOnce(() -> s_SuperSubsystem.level3State()).alongWith(s_SuperSubsystem.runSubsystems()));
        xboxController.back().onTrue(Commands.runOnce(() -> s_SuperSubsystem.zeroState()).alongWith(s_SuperSubsystem.runSubsystems()));
        xboxController.leftBumper().onTrue(Commands.runOnce(() -> s_SuperSubsystem.level4State()).alongWith(s_SuperSubsystem.runSubsystems()));
        xboxController.rightBumper().onTrue(Commands.runOnce(() -> s_SuperSubsystem.lowerAlgeaState()).alongWith(s_SuperSubsystem.runSubsystems()));
        xboxController.leftStick().onTrue(Commands.runOnce(() -> s_SuperSubsystem.upperAlgeaState()).alongWith(s_SuperSubsystem.runSubsystems()));
        xboxController.rightStick().onTrue(Commands.runOnce(() -> s_SuperSubsystem.climbState()).alongWith(s_SuperSubsystem.runSubsystems()));
        autoTrackButton.whileTrue(new AutoTeleop(s_Swerve, s_Vision));
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
