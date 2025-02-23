// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.MrPibb;

public enum MrPibbStates {
    //TODO: Update max and setpoints with new zero and values.
    //16.58 Max for Wrist
    GROUND_CORAL(16.7, 0),
    GROUND_ALGAE(9.33, -3.6),
    PROCESSOR(10.435, -3.62),
    BARGE(19.5, 0),
    CLIMB(9.8, -1.7),
    STATION(-1, -3.55),
    L1(17.3, 0),
    L2(9, -1.76),
    L3(11.9, -1.731),
    L4(16.8, -1.701),
    PROTECT(3.5, 0),
    CORAL_PRESTAGE(9.788, -1.824),
    LOWER_ALGAE(14.3, -3.6),
    UPPER_ALGAE(14.5, -3.5);

    private final double wrist, turret;

    public static final double NEUTRAL_WRIST = 3.5;

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
