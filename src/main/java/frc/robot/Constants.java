package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.generated.CompTunerConstants;
import frc.robot.generated.PracticeTunerConstants;

public final class Constants {
    public static final int CANDLE_ID = 26;
    public static final int PDH_ID = 27;
    public static final int NUM_LEDS = 8;
    public static final String CANIVORE_BUS = "CANivore";
    public static final String COMP_SERIAL_NUM = "032D2164";
    public static final boolean PRACTICE_BOT = !RobotController.getSerialNumber().equals(COMP_SERIAL_NUM);
    public static final boolean USE_LOGGER = true;
    public static final boolean DEBUG = false;

    public static final class Controllers {
        public static final GenericHID TRANSLATION_CONTROLLER = new GenericHID(0);
        public static final GenericHID ROTATION_CONTROLLER = new GenericHID(1);
        public static final GenericHID XBOX_CONTROLLER = new GenericHID(2);
        public static final GenericHID BUTTONS = new GenericHID(3);
        public static final double STICK_DEADBAND = 0.1;
        public static final int TRANSLATION_BUTTON = 1;
        public static final int ROTATION_BUTTON = 1;
        public static final int TRANSLATION_AXIS = 1;
        public static final int STRAFE_AXIS = 0;
        public static final int ROTATION_AXIS = 0;
    }

    public static final class Vision {
        public static final String FRONT_CAMERA_NAME = "FrontCamera";
        public static final String BACK_LEFT_CAMERA_NAME = "BackLeftCamera";
        public static final String BACK_RIGHT_CAMERA_NAME = "BackRightCamera";
        public static final String CORAL_CAMERA_NAME = "CoralCamera";
        public static final Transform3d FRONT_CAMERA_TRANSFORM = new Transform3d(new Translation3d(-0.022582, Inches.of((3.0/16.0)).in(Meters), 0.185/*0.138568*/), new Rotation3d(0, -0.174533, 0));
        public static final Transform3d BACK_LEFT_CAMERA_TRANSFORM = new Transform3d(new Translation3d(-0.43945, 0.3477, 0.188698), new Rotation3d(0, -0.174533, 3.6909));
        public static final Transform3d BACK_RIGHT_CAMERA_TRANSFORM = new Transform3d(new Translation3d(-0.43945, -0.3477, 0.188698), new Rotation3d(0, -0.174533, 2.59225));
        public static final Vector<N3> TAG_VISION_STDS =  VecBuilder.fill(0.1, 0.1, 9999999);//VecBuilder.fill(0.846, 0.364, 9999999);
    }

    public static final class Swerve {
        public static final double MAX_SPEED = (Constants.PRACTICE_BOT) ? PracticeTunerConstants.kSpeedAt12Volts.in(MetersPerSecond) : CompTunerConstants.kSpeedAt12Volts.in(MetersPerSecond); 
        public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(1.25).in(RadiansPerSecond);
    }

    public static final class Soda {

        public static final class DrPepper {
            public static final int LOADER_CAN_ID = 5; 
            public static final int THUMB_CAN_ID = 2; 
            public static final int SHERLOCK_CAN_ID = 28;
            public static final int WATSON_CAN_ID = 29;
        }

        public static final class MrPibb {
            public static final int WRIST_CAN_ID = 3; 
            public static final int TURRET_CAN_ID = 4;
        }
    }
    
    public static final class ClimbAvator {
        public static final int SHOULDER_CAN_ID = 12; 
        public static final int BOULDER_CAN_ID = 13; 
        public static final int ELEVATOR_CAN_ID = 14; 
        public static final int DETONATOR_CAN_ID = 15; 
        public static final int BILBO_BAGGINS_THE_BACK = 16;
    }
}