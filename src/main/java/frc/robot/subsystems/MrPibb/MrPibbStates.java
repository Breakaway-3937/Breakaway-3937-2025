// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.MrPibb;

public enum MrPibbStates {
    //TODO: Update max and setpoints with new zero and values.
    //16.58 Max for Wrist when elevator not extended.
    GROUND_CORAL(16.7, 0),
    GROUND_ALGAE(11.5, -3.5),
    PROCESSOR(10.435, -3.62),
    BARGE(19.5, 0),
    CLIMB(-2, 0),
    STATION(-1.15, -3.55),
    L1(17.3, 0),
    L2(10.3, -1.7),
    L3(11.9, -1.7),
    L4(16.8, -1.7),
    PROTECT(3.5, 0),
    CORAL_PRESTAGE(9.788, -1.7),
    LOWER_ALGAE(16, -3.5),
    UPPER_ALGAE(16, -3.5);

    private final double wrist, turret;

    public static final double NEUTRAL_WRIST = 4.25;

    private MrPibbStates(double wrist, double turret) {
        this.wrist = wrist;
        this.turret = turret;
    } 

    public double getWrist() {
        return wrist;
    }

    public double getTurret() {
        return turret;
    }

    public static double getNeutralWrist() {
        return NEUTRAL_WRIST;
    }
}
