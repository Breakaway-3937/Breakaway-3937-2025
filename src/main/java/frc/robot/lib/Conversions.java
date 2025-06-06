package frc.robot.lib;

public class Conversions { 
    private static final double shoulderConversion = 90.0 / 55.90825; //Degrees / Motor rotations to reach degree in numerator
    private static final double turretConversion = 180.0 / -3.6;

    public static double shoulderDegreesToRotations(double degree) {
        return degree / shoulderConversion;
    }

    public static double turretDegreeToRotations(double degree) {
        return degree / turretConversion;
    }
    
    //L2 3.78in, L3 16 5/8 in
}
