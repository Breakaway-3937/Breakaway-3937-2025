// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.DriveToPose;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final Joystick translationController = new Joystick(Constants.Controllers.TRANSLATION_CONTROLLER.getPort());
    private final Joystick rotationController = new Joystick(Constants.Controllers.ROTATION_CONTROLLER.getPort());

    JoystickButton translationButton = new JoystickButton(translationController, Constants.Controllers.TRANSLATION_BUTTON);
    JoystickButton rotationButton = new JoystickButton(rotationController, Constants.Controllers.ROTATION_BUTTON);

    private final int translationAxis = Constants.Controllers.TRANSLATION_AXIS;
    private final int strafeAxis = Constants.Controllers.STRAFE_AXIS;
    private final int rotationAxis = Constants.Controllers.ROTATION_AXIS;

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final Vision vision = new Vision();
    public final Swerve drivetrain = TunerConstants.createDrivetrain();

    Trigger rotation = new Trigger(() -> rotationController.getRawAxis(rotationAxis) == 0);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(translationController.getRawAxis(translationAxis) * MaxSpeed)
                    .withVelocityY(translationController.getRawAxis(strafeAxis) * MaxSpeed) 
                    .withRotationalRate(rotationController.getRawAxis(rotationAxis)  * MaxAngularRate)
            )
        );

        //translationButton.onTrue(Commands.runOnce(() -> drivetrain.setGyro(), drivetrain));

        translationButton.whileTrue(new DriveToPose(drivetrain, new Pose2d(7, 7, new Rotation2d(0))));


        /*joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));



        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d());
        }*/
        //translationButton.onFalse(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public RobotContainer() {
        configureBindings();
    }

    public Command getAutonomousCommand() {
        return AutoBuilder.buildAuto("New Auto");
    }
}
