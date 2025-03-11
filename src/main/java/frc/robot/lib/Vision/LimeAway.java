// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.Vision;

/** Limelight helper class but better. */
public class LimeAway {
    private final String name;

    public LimeAway(String name) {
        this.name = name;
    }

    /**
     * Gets the horizontal offset from the crosshair to the target in degrees.
     */
    public double getTX() {
        return LimelightHelpers.getTX(name);
    }

    /**
     * Gets the vertical offset from the crosshair to the target in degrees.
     */
    public double getTY() {
        return LimelightHelpers.getTY(name);
    }

    /**
     * Gets the target area as a percentage of the image (0-100%).
     */
    public double getAreaOfTarget() {
        return LimelightHelpers.getTA(name);
    }

    /**
     * Gets the primary neural detector result class name.
     */
    public String getDetectorClass() {
        return LimelightHelpers.getDetectorClass(name);
    }

    /**
     * Gets the detector class index from the primary result of the currently running neural detector pipeline.
     */
    public int getDetectorIndex() {
        return LimelightHelpers.getDetectorClassIndex(name);
    }

    public boolean hasTarget() {
        return LimelightHelpers.getTV(name);
    }

    public void turnLedsOff() {
        LimelightHelpers.setLEDMode_ForceOff(name);
    }
}
