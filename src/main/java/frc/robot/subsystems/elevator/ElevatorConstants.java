package frc.robot.subsystems.elevator;

import lib.PIDProfile;
import lib.hardware.motor.PortConfig;

public class ElevatorConstants {
    public static final PortConfig MOTOR_CONFIG = new PortConfig(20, false);
    public static final PIDProfile RAISE_GAINS = new PIDProfile().setP(0.1).setV(0.05).setD(0.0015).setSlot(0);
    public static final PIDProfile LOWER_GAINS = new PIDProfile().setP(0.1).setV(0.05).setD(0.0015).setSlot(1);

    /**
     * Motor Input to Elevator Output Ratio
     */
    public static final double PULLEY_CIRCUMFERENCE_INCHES = 2.0 * Math.PI;
    public static final double GEAR_RATIO = 1.0 / (125.0 * 2.0);

}