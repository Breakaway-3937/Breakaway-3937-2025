// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//import frc.robot.commands.Music;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Swerve.AutoPathLocations;
import frc.robot.subsystems.Swerve.Swerve;

public class RobotContainer {
    //FIXME: Add the correct max speed and max angular rate.
    private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); 
    private final double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final Joystick translationController = new Joystick(Constants.Controllers.TRANSLATION_CONTROLLER.getPort());
    private final Joystick rotationController = new Joystick(Constants.Controllers.ROTATION_CONTROLLER.getPort());
    private final CommandXboxController xboxController = new CommandXboxController(Constants.Controllers.XBOX_CONTROLLER.getPort());
    private final Joystick buttons = new Joystick(Constants.Controllers.BUTTONS.getPort());

    private final JoystickButton translationButton = new JoystickButton(translationController, Constants.Controllers.TRANSLATION_BUTTON);
    private final JoystickButton rotationButton = new JoystickButton(rotationController, Constants.Controllers.ROTATION_BUTTON);
    private final JoystickButton autoTrackButton = new JoystickButton(buttons, 1);

    private final int translationAxis = Constants.Controllers.TRANSLATION_AXIS;
    private final int strafeAxis = Constants.Controllers.STRAFE_AXIS;
    private final int rotationAxis = Constants.Controllers.ROTATION_AXIS;

    private final Telemetry logger = new Telemetry(maxSpeed);
    private final SendableChooser<Command> autoChooser;

    private final Swerve s_Swerve = TunerConstants.createDrivetrain();
    private final Vision s_Vision = new Vision(s_Swerve);

    private Trigger levelOne = OperatorController.getLevelOneTrigger();
    private Trigger levelTwo = OperatorController.getLevelTwoTrigger();
    private Trigger levelThree = OperatorController.getLevelThreeTrigger();
    private Trigger levelFour = OperatorController.getLevelFourTrigger();
    private Trigger noLevel = OperatorController.getNoLevelTrigger();
    private Trigger coralA = OperatorController.getCoralATrigger();
    private Trigger coralB = OperatorController.getCoralBTrigger();
    private Trigger coralC = OperatorController.getCoralCTrigger();
    private Trigger coralD = OperatorController.getCoralDTrigger();
    private Trigger coralE = OperatorController.getCoralETrigger();
    private Trigger coralF = OperatorController.getCoralFTrigger();
    private Trigger coralG = OperatorController.getCoralGTrigger();
    private Trigger coralH = OperatorController.getCoralHTrigger();
    private Trigger coralI = OperatorController.getCoralITrigger();
    private Trigger coralJ = OperatorController.getCoralJTrigger();
    private Trigger coralK = OperatorController.getCoralKTrigger();
    private Trigger coralL = OperatorController.getCoralLTrigger();
    private Trigger noTarget = OperatorController.getNoTargetTrigger();

    //private final Music c_Music = new Music();

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * Constants.Controllers.STICK_DEADBAND)
            .withRotationalDeadband(maxAngularRate * Constants.Controllers.STICK_DEADBAND) 
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private void configureBindings() {

        s_Swerve.setDefaultCommand(
            s_Swerve.applyRequest(() ->
                drive.withVelocityX(translationController.getRawAxis(translationAxis) * maxSpeed)
                    .withVelocityY(translationController.getRawAxis(strafeAxis) * maxSpeed) 
                    .withRotationalRate(rotationController.getRawAxis(rotationAxis)  * maxAngularRate)
            )
        );

        levelOne.onTrue(new PrintCommand("Level One"));
        levelTwo.onTrue(new PrintCommand("Level Two"));
        levelThree.onTrue(new PrintCommand("Level Three"));
        levelFour.onTrue(new PrintCommand("Level Four"));
        noLevel.onTrue(new PrintCommand("No Level"));
        coralA.onTrue(new PrintCommand("Coral A"));
        coralB.onTrue(new PrintCommand("Coral B"));
        coralC.onTrue(new PrintCommand("Coral C"));
        coralD.onTrue(new PrintCommand("Coral D"));
        coralE.onTrue(new PrintCommand("Coral E"));
        coralF.onTrue(new PrintCommand("Coral F"));
        coralG.onTrue(new PrintCommand("Coral G"));
        coralH.onTrue(new PrintCommand("Coral H"));
        coralI.onTrue(new PrintCommand("Coral I"));
        coralJ.onTrue(new PrintCommand("Coral J"));
        coralK.onTrue(new PrintCommand("Coral K"));
        coralL.onTrue(new PrintCommand("Coral L"));
        noTarget.onTrue(new PrintCommand("No Target"));

        translationButton.onTrue(Commands.runOnce(() -> s_Swerve.seedFieldCentric(), s_Swerve));

        //FIXME: Add full logic for autonomous tracking.
        autoTrackButton.whileTrue(s_Swerve.pathFindThenFollow(AutoPathLocations.CORAL_ONE, new Pose2d(5.774, 1.937, new Rotation2d())))
                        .whileFalse(Commands.none());

        s_Swerve.registerTelemetry(logger::telemeterize);
    }

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("DO NOTHING");
        autoChooser.addOption("L4 Left", new PathPlannerAuto("L4 Right", true));
        Shuffleboard.getTab("Auto").add(autoChooser).withPosition(0, 0).withSize(2, 1);
        configureBindings();
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public Vision getVisionSystem() {
        return s_Vision;
    }

    public Swerve getSwerveSystem() {
        return s_Swerve;
    }

    //FIXME: Fix music.
    //public Command getMusicCommand() {
    //    return c_Music;
    //}

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
