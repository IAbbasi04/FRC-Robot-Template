package frc.robot.subsystems.io;

import lib.team8592.Utils;

public abstract class LinearSubsystemIO implements ISubsystemIO {
    protected double desiredPosition = 0d;

    public abstract void setPosition(double units);

    public abstract double getPosition();

    public abstract boolean atPosition();

    public double getDesiredPosition() {
        return desiredPosition;
    }

    public boolean atPosition(double position, double tolerance) {
        return Utils.isWithin(getPosition(), position, tolerance);
    }

    @Override
    public void halt() {
        setPosition(getPosition());
    }
}