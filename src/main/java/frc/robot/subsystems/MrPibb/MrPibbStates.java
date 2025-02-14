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
    CLIMB(9.8, -1.7, 0),
    STATION(-1.523438, -3.598, 0),
    L1(13.2, 0, 0),
    L2(10.7, -1.83, 0),
    L3(10.851, -1.842, 0),
    L4(18.767578, -1.842, 0),
    PROTECT(0, 0, 0),
    CORAL_PRESTAGE(9.788, -1.824, 0),
    LOWER_ALGAE(11.8, -3.5, 0),
    UPPER_ALGAE(11.8, -3.5, 0),
    //TODO
    CLIMB_TEST(9.8, -1.7, 0);
    //Wrist Max = 15.767578
    private final double wrist, turret, speed;

    public static final double NEUTRAL_WRIST = 2.3;

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
