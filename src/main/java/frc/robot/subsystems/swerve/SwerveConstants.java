package frc.robot.subsystems.swerve;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.swerve.ctre.BaseTunerConstants;
import lib.PIDProfile;

public class SwerveConstants {
    protected static final double MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND = BaseTunerConstants.kSpeedAt12Volts.baseUnitMagnitude(); // m/s
    protected static final double MAX_ROTATIONAL_VELOCITY_RADIANS_PER_SECOND = Math.toRadians(720); // rad/s
    protected static final double MAX_TRANSLATIONAL_ACCELERATION_METERS_PER_SECOND_SQUARED = 3.0; // m/s^2
    protected static final double MAX_ROTATIONAL_ACCELERATION_METERS_PER_SECOND_SQUARED = Math.toRadians(720); // rad/s^2

    protected static final PIDProfile SNAP_TO_GAINS = 
        new PIDProfile()
            .setP(2d)
            .setD(0.1)
            .setTolerance(0.15)
            .setContinuousInput(-180, 180);

    protected static final PIDController SNAP_TO_PID = SNAP_TO_GAINS.toPIDController();

    protected static final PathConstraints DRIVE_TO_POSE_CONSTRAINTS = new PathConstraints(
        MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND, 
        MAX_ROTATIONAL_VELOCITY_RADIANS_PER_SECOND,
        MAX_ROTATIONAL_VELOCITY_RADIANS_PER_SECOND, 
        MAX_ROTATIONAL_ACCELERATION_METERS_PER_SECOND_SQUARED
    );

    protected static final PIDProfile PATH_FOLLOW_TRANSLATE_GAINS = new PIDProfile().setP(10d).setTolerance(0.1);
    protected static final PIDProfile PATH_FOLLOW_ROTATE_GAINS = new PIDProfile()
        .setP(6d).setD(0.1)
        .setMaxVelocity(4*Math.PI)
        .setMaxAcceleration(4*Math.PI)
        .setTolerance(0.1)
        .setContinuousInput(-Math.PI, Math.PI)
    ;

    protected static final double DEFAULT_TRANSLATIONAL_SCALING = 1.0; 
    protected static final double SNAIL_MODE_TRANSLATIONAL_SCALING = 0.5;

    protected static final double DEFAULT_ROTATION_SCALING = 0.75; 
    protected static final double SNAIL_MODE_ROTATION_SCALING = 0.3;

    protected static final double JOYSTICK_DEADZONE = 0.03;
}