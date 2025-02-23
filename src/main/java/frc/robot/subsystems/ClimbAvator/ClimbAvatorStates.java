// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ClimbAvator;

public enum ClimbAvatorStates {
    //57.251 Max for Shoulder
    GROUND_CORAL(4.77, 0.25),
    GROUND_ALGAE(5.3, 7.6), //TODO
    PROCESSOR(0.25, 17.825),
    BARGE(73.5, 50),
    CLIMB(0.25, 48),
    STATION(0.25, 50),
    L1(0.25, 21.3),
    L2(5, 32.75), //TODO
    L3(27.98, 41.5), //TODO
    L4(73.5, 47.475), //TODO
    PROTECT(0.25, 0.25),
    CORAL_PRESTAGE(0.25, 40),
    LOWER_ALGAE(12, 35),
    UPPER_ALGAE(36.1, 39.9),
    CLIMB_PULL(0.25, 7.5);

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
