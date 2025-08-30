package frc.robot.subsystems.pivot;

import lib.PIDProfile;
import lib.hardware.motor.PortConfig;

public class PivotConstants {
    public static final PortConfig MOTOR_CONFIG = new PortConfig(21, false);

    public static final PIDProfile RAISE_GAINS = new PIDProfile()
        .setP(0.1)
        .setV(0.05)
        .setD(0.0015)
        .setMaxVelocity(4000)
        .setMaxAcceleration(6000)
        .setSlot(0);

    public static final PIDProfile LOWER_GAINS = new PIDProfile()
        .setP(0.1)
        .setV(0.05)
        .setD(0.0015)
        .setMaxVelocity(3000)
        .setMaxAcceleration(3000)
        .setSlot(1);
}