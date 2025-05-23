package frc.robot.subsystems.io;

import lib.team8592.Utils;

public interface ILinearSubsystemIO extends ISubsystemIO {
    public void setPosition(double units);

    public double getPosition();

    public boolean atPosition();

    public double getDesiredPosition();

    public default boolean atPosition(double position, double tolerance) {
        return Utils.isWithin(getPosition(), position, tolerance);
    }
}