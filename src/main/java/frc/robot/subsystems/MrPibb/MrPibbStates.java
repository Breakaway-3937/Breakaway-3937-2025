// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.MrPibb;

public enum MrPibbStates {
    FLOOR_INTAKE_CORAL(0, 0, 0),
    FLOOR_INTAKE_ALGAE(0, 0, 0),
    STOW_MODE(0, 0, 0),
    PROCESSOR(0, 0, 0),
    BARGE(0, 0, 0),
    CLIMB(0, 0, 0),
    STATION(-2.715, -3.598, 0),
    L1(13.648, 0, 0),
    L2(9.490, -1.891, 0),
    L3(0, 0, 0),
    L4(0, 0, 0),
    PROTECT(0, 0, 0),
    CORAL_PRESTAGE(9.788, -1.824, 0);

    private final double wrist, turret, speed;

    public static final double NEUTRAL_WRIST = 0;

    private MrPibbStates(double wrist, double turret, double speed) {
        this.wrist = wrist;
        this.turret = turret;
        this.speed = speed;
    } 

    public double getWrist() {
        return wrist;
    }

    public double getTurret() {
        return turret;
    }

    public double getSpeed() {
        return speed;
    }

    public static double getNeutralWrist() {
        return NEUTRAL_WRIST;
    }
}
