package frc.robot.subsystems.roller;

import lib.PIDProfile;
import lib.hardware.motor.PortConfig;

public class RollerConstants {
    public static final PortConfig ROLLER_CONFIG = new PortConfig(10, false);

    public static final PIDProfile ROLLER_GAINS = new PIDProfile().setP(1E-3).setV(1E-2).setSlot(0);
}