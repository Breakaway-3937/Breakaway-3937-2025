package frc.robot.lib;

public class Conversions { 
    private static final double shoulderConversion = 90.0 / -0.223633; //Degrees / Motor rotations to reach degree in numerator
    private static final double turretConversion = 90.0 / -1.85;

    public static double shoulderDegreesToRotations(double degree) {
        return degree / shoulderConversion;
    }

    public static double turretDegreeToRotations(double degree) {
        return degree / turretConversion;
    }
}
