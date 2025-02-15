// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.MrPibb;

public enum MrPibbStates {
    //Wrist Max = 16.58
    FLOOR_INTAKE_CORAL(16.2, 0, 0),
    FLOOR_INTAKE_ALGAE(0, 0, 0),
    STOW_MODE(0, 0, 0),
    PROCESSOR(10.435, -3.62, 0),
    BARGE(0, 0, 0),
    CLIMB(9.8, -1.7, 0),
    STATION(-0.523438, -3.598, 0),
    L1(13.2, 0, 0),
    L2(10.7, -1.83, 0),
    L3(10.851, -1.842, 0),
    L4(16.5, -1.842, 0),
    PROTECT(4.6, 0, 0),
    CORAL_PRESTAGE(9.788, -1.824, 0),
    LOWER_ALGAE(11.8, -3.5, 0),
    UPPER_ALGAE(11.8, -3.5, 0);

    private final double wrist, turret, speed;

    public static final double NEUTRAL_WRIST = 4.6;

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
