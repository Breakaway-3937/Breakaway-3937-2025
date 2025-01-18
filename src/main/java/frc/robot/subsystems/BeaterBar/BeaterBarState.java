// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.BeaterBar;

/** Add your docs here. */
public enum BeaterBarState {
    GROUND_PICKUP(0, 0),
    STORE(0, 0),
    REST(0, 0);

    private double position;
    private double speed;

    private BeaterBarState(double position, double speed) {
        this.position = position;
        this.speed = speed;
    }

    public double getPosition() {
        return position;
    }

    public double getSpeed() {
        return speed;
    }
}