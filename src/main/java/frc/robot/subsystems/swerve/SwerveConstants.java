package frc.robot.subsystems.swerve;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swerve.ctre.BaseTunerConstants;
import lib.PIDProfile;

public class SwerveConstants {
    public static final double MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND = BaseTunerConstants.kSpeedAt12Volts.baseUnitMagnitude(); // m/s
    public static final double MAX_ROTATIONAL_VELOCITY_RADIANS_PER_SECOND = Math.toRadians(720); // rad/s

    public static final PIDProfile SNAP_TO_GAINS = new PIDProfile().setP(2d).setD(0.1).setTolerance(0.15).setContinuousInput(-180, 180);

    public static final PIDProfile DRIVE_TO_POSE_GAINS = 
        new PIDProfile()
            .setP(2.5d)
            .setD(0.01d)
            .setMaxVelocity(3d)
            .setMaxAcceleration(3d)
            .setTolerance(0.02);

    public static final PathConstraints DRIVE_TO_POSE_CONSTRAINTS = new PathConstraints(
        MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND, 
        3.0,
        Units.degreesToRadians(720), 
        Units.degreesToRadians(720)
    );

    public static final PIDProfile PATH_FOLLOW_TRANSLATE_GAINS = new PIDProfile().setP(10d).setTolerance(0.1);
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

    public static final double JOYSTICK_DEADZONE = 0.03;
}