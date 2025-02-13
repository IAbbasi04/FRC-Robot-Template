package org.team8592.frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
    public static final String CAM_NAME = "Arducam_OV9782_E";

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