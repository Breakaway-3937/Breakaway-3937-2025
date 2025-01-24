// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ClimbAvator;

public enum ClimbAvatorStates {
    FLOOR_INTAKE_CORAL(0, 0),
    FLOOR_INTAKE_ALGAE(0, 0),
    STOW_MODE(0, 0),
    PROCESSOR(0, 0),
    BARGE(0, 0),
    CLIMB(0, 0),
    STATION(0, 0),
    L1(0, 0),
    L2(0, 0),
    L3(0, 0),
    L4(0, 0),
    PROTECT(0, 0);

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
