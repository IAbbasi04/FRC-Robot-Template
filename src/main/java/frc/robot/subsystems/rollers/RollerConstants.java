package frc.robot.subsystems.rollers;

import lib.PIDProfile;
import lib.hardware.motor.PortConfig;

public class RollerConstants {
    public static final PortConfig MOTOR_CONFIG = new PortConfig(20, false);
    public static final PIDProfile VELOCITY_GAINS = new PIDProfile().setP(0.1).setV(0.15).setSlot(0);
}