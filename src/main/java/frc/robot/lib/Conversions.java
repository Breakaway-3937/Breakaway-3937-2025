package frc.robot.lib;

public class Conversions {
    private static double shoulderConversion = 0; //Degrees / Motor rotations to reach degree in numerator
    private static double elevatorConversion = 0;
    private static double wristConversion = 0;
    private static double turretConversion = 0;

    public static double shoulderDegreesToRotations(double degree) {
        return degree / shoulderConversion;
    }

    public static double elevatorInchesToRotations(double inches) {
        return inches / elevatorConversion;
    }

    public static double wristDegreeToRotations(double degree) {
        return degree / wristConversion;
    }

    public static double turretDegreeToRotations(double degree) {
        return degree / turretConversion;
    }
}
