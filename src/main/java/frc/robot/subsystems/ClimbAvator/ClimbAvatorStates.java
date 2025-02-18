// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ClimbAvator;

public enum ClimbAvatorStates {
    //-0.229004 Max for Shoulder
    GROUND_CORAL(4.77, 0),
    GROUND_ALGAE(0.25, -0.0319),
    PROCESSOR(0.25, -0.0713),
    BARGE(70, -0.2), //TODO
    CLIMB(0.25, -0.192),
    //STATION(0.25, -0.196),
    STATION(0.25, -0.1),
    L1(0.25, -0.0852),
    L2(6.2, -0.139),
    //L3(18, -0.1689),
    L3(19.7, -0.1953),
    L4(70, -0.2),
    PROTECT(0.25, 0),
    CORAL_PRESTAGE(0.25, -0.16),
    LOWER_ALGAE(12, -0.14),
    UPPER_ALGAE(36.1, -0.1596),
    CLIMB_PULL(0.25, 0);

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
