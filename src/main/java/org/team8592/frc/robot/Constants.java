package org.team8592.frc.robot;

import edu.wpi.first.math.geometry.*;

public final class Constants {
    public final class SHARED {
        public static final String LOG_FOLDER = "CustomLogs";
    }
    public final class CONVERSIONS {
        public static final double METERS_SECOND_TO_TICKS_TALONFX = ((2048 * 6.75 * 60) / (200 * Math.PI * 0.0508));

        public static final double RPM_TO_TICKS_100_MS_TALONFX = 2048.0 / 600.0;

        public static final double ANGLE_DEGREES_TO_TICKS_SPARKFLEX = 4096 / 360.0;
        public static final double TICKS_TO_ANGLE_DEGREES_SPARKFLEX = 360.0 / 4096.0;

        public static final double DEG_TO_RAD = 0.0174533;
        public static final double RAD_TO_DEG = 57.2958;
        public static final double IN_TO_METERS = 0.0254;
        public static final double METERS_TO_FEET = 3.28084;
    }

    public final class MEASUREMENTS {
        public static final double FIELD_LENGTH_METERS = 17.548;
        public static final double FIELD_WIDTH_METERS = 8.052;
    }

    public final class CONTROLLERS {
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;
        public static final int CORAL_SELECTOR_PORT = 2;

        // Assignments for coralController buttons
        public static final int CORAL_CONTROLLER_L1 = 7;
        public static final int CORAL_CONTROLLER_L2 = 8;
        public static final int CORAL_CONTROLLER_L3 = 2;
        public static final int CORAL_CONTROLLER_L4 = 3;
        public static final int CORAL_CONTROLLER_R1 = 5;
        public static final int CORAL_CONTROLLER_R2 = 6;
        public static final int CORAL_CONTROLLER_R3 = 1;
        public static final int CORAL_CONTROLLER_R4 = 4;
    }

    public final class POWER {
        public static final int SWERVE_MAX_VOLTAGE = 12;
        public static final int SWERVE_DRIVE_CURRENT_LIMIT = 80;
        public static final int SWERVE_STEER_CURRENT_LIMIT = 40;
    }

    public final class VISION {
        public static final String CAM_NAME = "Arducam_OV9782_E";

        public static final double X_KP = 1; // Used to be 1. Currently testing x_KP of 0
        public static final double X_KI = 0;
        public static final double X_KD = 0.01;

        public static final double Y_KP = 0.8; // Originally set to 0.5
        public static final double Y_KI = 0;
        public static final double Y_KD = 0;

        public static final double ROT_KP = 0.01; // 0.015 was the original value but set to 0 for testing y axis
        public static final double ROT_KI = 0;
        public static final double ROT_KD = 0.0001;

        public static final double OFFSET_DEPTH = 0.50;
        public static final double SPEED_SCALE = 1.0;
        public static final double SPEED_MAX = 0.2;

        public static final int MAX_LOCK_LOSS_TICKS = 20;

        // Our camera is mounted 0.6 meters forward and 0.05 meters up from the robot pose,
        // (Robot pose is considered the center of rotation at the floor level, or Z = 0)
        // and pitched 15 degrees up.
        public static final Transform3d CAMERA_OFFSET = new Transform3d();

        public static final double MAX_ACCEPTABLE_AMBIGUITY = 0.2;
    }

    public final class ROBOT {
        public static final String LOG_PATH = SHARED.LOG_FOLDER + "/Robot/";
    }

    public class SUPPLIERS{
        public static final String LOG_PATH = SHARED.LOG_FOLDER+"/Suppliers/";
    }
}