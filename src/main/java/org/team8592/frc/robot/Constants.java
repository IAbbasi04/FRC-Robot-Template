package org.team8592.frc.robot;

import org.team8592.frc.robot.RobotSelector.RobotType;
import org.team8592.lib.PIDProfile;

import edu.wpi.first.math.geometry.Transform3d;

public class Constants {
    public class CONFIG {
        public static final RobotType ROBOT_TYPE = RobotType.COMP_BOT;
        public static final boolean IGNORE_SIMBOT = false; // Runs the selected robot type in simulation rather than the simbot
        public static final String GAME = "[_]";
        public static final String YEAR = "202[_]";
        public static final String ROBOT = "[_]";
        public static final String TEAM = "8592";
    }

    public class SWERVE {
        public static final PIDProfile SNAP_TO_GAINS = new PIDProfile().setP(3.7).setD(0.1).setTolerance(0.1);
        public static final PIDProfile PATH_FOLLOW_TRANSLATE_GAINS = new PIDProfile().setP(8d).setTolerance(0.1);
        public static final PIDProfile PATH_FOLLOW_ROTATE_GAINS = new PIDProfile()
            .setP(6d).setD(0.1)
            .setMaxVelocity(4*Math.PI)
            .setMaxAcceleration(4*Math.PI)
            .setTolerance(0.1)
            .setContinuousInput(-Math.PI, Math.PI)
        ;

        public static final double TRANSLATE_POWER_FAST = 1.0; 
        public static final double TRANSLATE_POWER_SLOW = 0.5;

        public static final double ROTATE_POWER_FAST = 0.75; 
        public static final double ROTATE_POWER_SLOW = 0.3;

        public static final int TRANSLATION_SMOOTHING_AMOUNT = 3;
        public static final int ROTATION_SMOOTHING_AMOUNT = 1;

        public static final double JOYSTICK_EXPONENT = 1.2;
    }

    public class ROLLER {
        
    }

    public class SHOULDER {
        public static final double SHOULDER_GEAR_RATIO = 1/180.0;

        public static final double SHOULDER_MIN_DEGREES = Suppliers.CURRENT_ROBOT.get().equals(RobotType.DEV_BOT) ? -2 : -2;
        public static final double SHOULDER_MAX_DEGREES = Suppliers.CURRENT_ROBOT.get().equals(RobotType.DEV_BOT) ? 180 : 180;

        public static final double SAFE_SHOULDER_TO_ROTATE_WRIST_DEGREES = Suppliers.CURRENT_ROBOT.get().equals(RobotType.DEV_BOT)  ? 75 : 40;

        public static final double SHOULDER_POSITION_TOLERANCE_DEGREES = 2.0;

        public static final int SHOULDER_CURRENT_LIMIT_AMPS = 40; //amps

        public static final double SHOULDER_MAX_ACCELERATION = 250;
        public static final double SHOULDER_MAX_VELOCITY = 100;

        public static final double SHOULDER_P = Suppliers.CURRENT_ROBOT.get().equals(RobotType.DEV_BOT) ? /*RIPTIDE: */3: /*PERRY: */3;
        public static final double SHOULDER_I = Suppliers.CURRENT_ROBOT.get().equals(RobotType.DEV_BOT) ? /*RIPTIDE: */0: /*PERRY: */0;
        public static final double SHOULDER_D = Suppliers.CURRENT_ROBOT.get().equals(RobotType.DEV_BOT) ? /*RIPTIDE: */0: /*PERRY: */0;
        public static final double SHOULDER_S = Suppliers.CURRENT_ROBOT.get().equals(RobotType.DEV_BOT) ? /*RIPTIDE: */0: /*PERRY: */0;
        public static final double SHOULDER_V = Suppliers.CURRENT_ROBOT.get().equals(RobotType.DEV_BOT) ? /*RIPTIDE: */0: /*PERRY: */0;
        public static final double SHOULDER_A = Suppliers.CURRENT_ROBOT.get().equals(RobotType.DEV_BOT) ? /*RIPTIDE: */0: /*PERRY: */0;
    }

    public class ELEVATOR {
        public static final double EXTENSION_GEAR_RATIO = 0.25;
        public static final double EXTENSION_DRUM_DIAMETER_INCHES = 1;

        public static final double EXTENSION_INCHES_MAX = Suppliers.CURRENT_ROBOT.get().equals(RobotType.DEV_BOT) ? /*RIPTIDE*/ 19.5: /*PERRY*/ 19.6; //this is in inches
        public static final double EXTENSION_INCHES_MIN = Suppliers.CURRENT_ROBOT.get().equals(RobotType.DEV_BOT) ? /*RIPTIDE*/ 0.5 : /*PERRY*/ 0.5;

        public static final double EXTENSION_POSITION_TOLERANCE_INCHES = 0.1;

        public static final int ELEVATOR_CURRENT_LIMIT_AMPS = 40;//amps

        public static final double ELEVATOR_MAX_ACCELERATION = 250;
        public static final double ELEVATOR_MAX_VELOCITY = 100; //formerly 100

        public static final double ELEVATOR_POSITION_P = Suppliers.CURRENT_ROBOT.get().equals(RobotType.DEV_BOT) ? /*RIPTIDE: */3.5: /*PERRY: */3.5;
        public static final double ELEVATOR_POSITION_I = Suppliers.CURRENT_ROBOT.get().equals(RobotType.DEV_BOT) ? /*RIPTIDE: */0: /*PERRY: */0;
        public static final double ELEVATOR_POSITION_D = Suppliers.CURRENT_ROBOT.get().equals(RobotType.DEV_BOT) ? /*RIPTIDE: */0: /*PERRY: */0;
        public static final double ELEVATOR_POSITION_S = Suppliers.CURRENT_ROBOT.get().equals(RobotType.DEV_BOT) ? /*RIPTIDE: */0: /*PERRY: */0;
        public static final double ELEVATOR_POSITION_V = Suppliers.CURRENT_ROBOT.get().equals(RobotType.DEV_BOT) ? /*RIPTIDE: */0: /*PERRY: */0;
        public static final double ELEVATOR_POSITION_A = Suppliers.CURRENT_ROBOT.get().equals(RobotType.DEV_BOT) ? /*RIPTIDE: */0: /*PERRY: */0;
    }

    public class WRIST {
        public static final double WRIST_GEAR_RATIO = 1/75.0;

        public static final double WRIST_ANGLE_DEGREES_MIN = CONFIG.ROBOT_TYPE.isCompBot()?-45d:-45d; // Currently the same between the 2 bots
        public static final double WRIST_ANGLE_DEGREES_MAX = CONFIG.ROBOT_TYPE.isCompBot()?220:220; // Currently the same between the 2 bots

        public static final double WRIST_POSITION_TOLERANCE = 2.0;

        public static final int WRIST_CURRENT_LIMIT = 60;//amps

        public static final double WRIST_MAX_ACCELERATION = 400;
        public static final double WRIST_MAX_VELOCITY = 100; //used to be 100
    }

    public class VISION {
        public static final String CAM_NAME = "Arducam_[NAME]";

        public static final Transform3d CAMERA_OFFSET = new Transform3d();

        public static final double MAX_ACCEPTABLE_AMBIGUITY = 0.20;
    }
}