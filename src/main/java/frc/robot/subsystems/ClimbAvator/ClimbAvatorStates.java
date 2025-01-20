// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ClimbAvator;

/** Add your docs here. */
public enum ClimbAvatorStates {
    STATION(0,0),
    L1(0,0),
    L2(0,0),
    L3(0,0),
    L4(0,0);

    private double height, angle;

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
