// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Soda;

import com.fasterxml.jackson.databind.introspect.TypeResolutionContext.Empty;

import frc.robot.Constants;

public enum MrPibbStates {
    //16.58 Max for wrist when elevator not extended.
    GROUND_CORAL(16.67, 0),
    GROUND_ALGAE(11.5, 3.5),
    PROCESSOR(10.435, 3.62),
    BARGE(19.5, 0),
    CLIMB(-1, 0),
    STATION(-1.15, 3.55),
    L1(17.3, 0),
    L2(10.3, 1.7),
    L3(11.9, 1.7),
    L4(17.6, 1.7),//was 17.4
    BACKWARDS_L3(11.9, 1.7),
    BACKWARDS_L4(17.6, 1.7),//was 17.4
    PROTECT(1.5, 0),
    CORAL_PRESTAGE(9.788, 1.7),
    EMPTY_PRESTATE(9.788, 0),
    LOWER_ALGAE(16, 3.5),
    UPPER_ALGAE(16, 3.5);

    private final double wrist, turret;

    private MrPibbStates(double wrist, double turret) {
        this.wrist = wrist;
        this.turret = turret;
    } 

    public double getWrist() {
        return Constants.PRACTICE_BOT ? wrist : wrist * 14.7 / 15.6;
    }

    public double getTurret() {
        return turret;
    }
}
