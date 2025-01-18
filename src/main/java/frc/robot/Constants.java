package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotController;

public final class Constants {
    public static final boolean ROBOT_RELATIVE = false;
    public static final boolean OPEN_LOOP = true;
    //FIXME: Will all change with new robot.
    public static final int CANDLE_ID = 0;
    public static final int PDH_ID = 0;
    public static final int NUM_LEDS = 0;
    public static final String CANIVORE_BUS = "CANivore";
    public static final String PRACTICE_SERIAL_NUM = "0324152B";
    public static final boolean PRACTICE_BOT = RobotController.getSerialNumber().equals(PRACTICE_SERIAL_NUM);
    public static final boolean DEBUG = true;

    public static final class Controllers{
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
        //FIXME: Will all change with new robot.
        public static final String FRONT_CAMERA_NAME = "Front Camera";
        public static final String BACK_CAMERA_NAME = "Global_Shutter_Camera";
        public static final String NOTE_CAMERA_NAME = "HD_USB_Camera";
        public static final Transform3d FRONT_CAMERA_TRANSFORM = new Transform3d(new Translation3d(-0.198, -0.172, 0.29), new Rotation3d(0.174533, 0, 0));
        public static final Transform3d BACK_CAMERA_TRANSFORM = new Transform3d(new Translation3d(-0.198, 0.172, 0.29), new Rotation3d(0.174533, 0, Math.PI));
        public static final Vector<N3> TAG_VISION_STDS_FRONT = VecBuilder.fill(0.716, 0.162, Units.degreesToRadians(10));
        public static final Vector<N3> TAG_VISION_STDS_BACK = VecBuilder.fill(1.294, 0.086, Units.degreesToRadians(10));
    }

    public static final class BeaterBar {
        public static final int ANKLE_CAN_ID = 0; //TODO 
        public static final int INTAKE_CAN_ID = 0; //TODO
    }

    public static final class EndEffector {
        public static final int WRIST_CAN_ID = 0; //TODO 
        public static final int LOADER_CAN_ID = 0; //TODO
    }
    
    public static final class ClimbAvator {
        public static final int SHOULDER_CAN_ID = 0;//TODO
        public static final int SHOULDER_FOLLOWER_CAN_ID = 0;//TODO
        public static final int ELEVATOR_CAN_ID = 0;//TODO
        public static final int ELEVATOR_FOLLOWER_CAN_ID = 0;//TODO
    }
}