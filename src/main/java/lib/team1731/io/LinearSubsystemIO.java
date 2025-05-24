package lib.team1731.io;

import lib.team8592.Utils;

public abstract class LinearSubsystemIO implements ISubsystemIO {
    protected double desiredPosition = 0.0;
    protected double positionTolerance = 0.0;

    protected LinearSubsystemIO(double positionTolerance) {
        this.positionTolerance = positionTolerance;
    }

    public abstract void setPosition(double units);

    public abstract double getPosition();

    public boolean atPosition() {
        return atPosition(getDesiredPosition());
    }

    public boolean atPosition(double target) {
        return Utils.isWithin(getPosition(), target, positionTolerance);
    }

    public double getDesiredPosition() {
        return this.desiredPosition;
    }
}