// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

public enum EndEffectorStates {
    STATION(0, 0),
    L1(0, 0),
    L2(0, 0),
    L3(0, 0),
    L4(0,0 );

    private double angle, speed;

    private EndEffectorStates(double angle, double speed) {
        this.angle = angle;
        this.speed = speed;
    } 

    public double getAngle() {
        return angle;
    }

    public double getSpeed() {
        return speed;
    }
}
