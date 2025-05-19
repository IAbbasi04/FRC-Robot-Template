package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;

public final class ElevatorConstants {
    public static final double DRUM_RADIUS_METERS = Units.inchesToMeters(1d);

    public static final double GEAR_RATIO = 1d / 25d;

    public static final double BASE_STAGE_HEIGHT_METERS = Units.inchesToMeters(30d);

    public static final double MAX_EXTENSION_METERS = Units.inchesToMeters(28d);

    public static final double CARRIAGE_MASS_KG = Units.lbsToKilograms(30d);

    public static final double POSITION_TOLERANCE_INCHES = 0.25; // inches
}