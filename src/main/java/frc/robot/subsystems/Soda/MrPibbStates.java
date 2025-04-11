// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Soda;

public enum MrPibbStates {
    GROUND_CORAL(19.1, 0),
    GROUND_ALGAE(21.2, 0),
    PROCESSOR(7.88, 0),
    BARGE(13.13, 0),
    BACKWARDS_BARGE(5.6, 0),
    CLIMB(0, 0),
    STATION(0.6, 0),
    L1(9, 0),
    L2(12.1 + 0.1, 1.83),
    L3(13.7 + 0.1, 1.83),
    L4(19.4 + 0.4, 1.83),
    POOPY_BACKWARDS_L4(16.5, 0),
    BACKWARDS_L2(4, 1.83),
    BACKWARDS_L3(7.5, 1.83),
    BACKWARDS_L4(2.1-0.15, 1.83),
    PROTECT(1.5, 0),
    CORAL_PRESTAGE(9.788, 1.83),
    EMPTY_PRESTATE(9.788, 0),
    LOWER_ALGAE(21.4, 0),
    UPPER_ALGAE(22.8, 0);

    private final double wrist, turret;

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
}
