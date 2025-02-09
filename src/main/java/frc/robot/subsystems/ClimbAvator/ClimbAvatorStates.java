// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ClimbAvator;

public enum ClimbAvatorStates {
    FLOOR_INTAKE_CORAL(0, 0),
    FLOOR_INTAKE_ALGAE(0, 0),
    STOW_MODE(0, 0),
    PROCESSOR(0, 0),
    BARGE(0, 0),
    CLIMB(0, 0),
    STATION(0, -49.0/250.0),
    L1(0, -0.079590),
    L2(6.2, -0.139),
    L3(24.5, -0.1689),
    L4(0, 0),
    PROTECT(0, 0),
    CORAL_PRESTAGE(0, -40.0/250.0);

    //MAX FOR 0-100 ON TURRET = 14.283 ON WRIST
    //MAX FOR 100-180 ON TURRET = 9.503
    //SHOULDER ON -24.585 FOR TURRET
    //MIN FOR 100-180 ON TURRET = -6.358

    private final double height, angle;

    public static final double NEUTRAL_ELEVATOR = 0;

    private ClimbAvatorStates(double height, double angle) {
        this.height = height;
        this.angle = angle;
    }
    
    public double getHeight() {
        return height;
    }

    public double getAngle() {
        return angle;
    }

    public static double getNeutralElevator() {
        return NEUTRAL_ELEVATOR;
    }
}
