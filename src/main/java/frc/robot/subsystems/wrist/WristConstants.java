package frc.robot.subsystems.wrist;

import lib.PIDProfile;
import lib.hardware.motor.PortConfig;

public class WristConstants {
    public static final PortConfig WRIST_MOTOR_CONFIG = new PortConfig(20, false);
    public static final PIDProfile WRIST_GAINS = new PIDProfile().setP(0.1).setV(0.05).setD(0.0015).setSlot(0);

    /**
     * Motor Input to Wrist Output Ratio
     */
    public static final double WRIST_GEAR_RATIO = 1.0 / (125.0 * 2.0);
}