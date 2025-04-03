// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ClimbAvator;

public enum ClimbAvatorStates {
    GROUND_CORAL(7.9, 0.25),
    GROUND_ALGAE(4.3, 12),
    PROCESSOR(1, 0.25),
    BARGE(73.5, 50),
    CLIMB(12.5, 51.3),
    STATION(1, 44),
    L1(1, 11),
    L2(2.88, 33.2),
    L3(22, 41.5),
    L4(73.5, 47.475),
    BACKWARDS_L2(1, 50.7),
    BACKWARDS_L3(18.5, 52.3),
    BACKWARDS_L4(73.2, 54.2),
    PROTECT(1, 0.25),
    CORAL_PRESTAGE(1, 40),
    LOWER_ALGAE(22, 34.6),
    UPPER_ALGAE(45.7, 39.3),
    CLIMB_PULL(18.9, 3.75);

    private final double height, angle;

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
}
