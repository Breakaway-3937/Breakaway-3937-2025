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
    STATION(0, -49.061),
    L1(0, -20.917),
    L2(0, -37.417),
    L3(0, 0),
    L4(0, 0),
    PROTECT(0, 0),
    CORAL_PRESTAGE(0, -40);

    //MAX FOR 0-100 ON TURRET = 14.283 ON WRIST
    //MAX FOR 100-180 ON TURRET = 9.503
    //SHOULDER ON -24.585 FOR TURRET
    //MIN FOR 100-180 ON TURRET = -6.358

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
