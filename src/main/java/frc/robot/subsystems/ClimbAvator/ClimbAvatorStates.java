// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ClimbAvator;

public enum ClimbAvatorStates {
    //57.251 Max for Shoulder
    GROUND_CORAL(6.3, 1.28),
    GROUND_ALGAE(7.6, 9.7),
    PROCESSOR(0.25, 19),
    BARGE(73.5, 50),
    CLIMB(12.5, 51.3),
    STATION(0.25, 48.5),
    L1(0.25, 22),
    L2(6, 33.2),
    L3(27, 41.5),
    L4(73.5, 47.475),
    BACKWARDS_L1(0.25, 22),
    BACKWARDS_L2(6, 33.2),
    BACKWARDS_L3(27, 41.5),
    BACKWARDS_L4(73.5, 47.475),
    PROTECT(0.25, 0.25),
    CORAL_PRESTAGE(0.25, 40),
    LOWER_ALGAE(21.7, 36),
    UPPER_ALGAE(44.3, 39.9),
    CLIMB_PULL(18.9, 3.75);

    private final double height, angle;

    public static final double NEUTRAL_SHOULDER = 25.879;

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

    public static double getNeutralShoulder() {
        return NEUTRAL_SHOULDER;
    }
}
