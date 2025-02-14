// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ClimbAvator;

public enum ClimbAvatorStates {
    //TODO: Update values
    //-0.229004 Max
    FLOOR_INTAKE_CORAL(0.25, 0),
    FLOOR_INTAKE_ALGAE(0.25, 0),
    STOW_MODE(0.25, 0),
    PROCESSOR(0.25, 0),
    BARGE(0.25, 0),
    CLIMB(0.25, 0),
    STATION(0.25, -0.196),
    L1(0.25, -0.079590),
    L2(6.2, -0.139),
    L3(24.5, -0.1689),
    L4(70, -0.2),
    PROTECT(0.25, 0),
    CORAL_PRESTAGE(0.25, -0.16),
    LOWER_ALGAE(13, -0.143),
    UPPER_ALGAE(39, -0.174053),
    //TODO
    CLIMB_TEST(0.25, -0.192);

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
