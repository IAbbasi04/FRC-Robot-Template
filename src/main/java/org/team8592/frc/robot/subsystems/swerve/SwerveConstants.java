package org.team8592.frc.robot.subsystems.swerve;

import org.team8592.frc.robot.subsystems.swerve.ctreswerve.TunerConstants;
import org.team8592.lib.PIDProfile;

public class SwerveConstants {
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

    public static final double MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND = TunerConstants.kSpeedAt12Volts.baseUnitMagnitude(); // m/s
    public static final double MAX_ROTATIONAL_VELOCITY_RADIANS_PER_SECOND = Math.toRadians(720); // rad/s
}