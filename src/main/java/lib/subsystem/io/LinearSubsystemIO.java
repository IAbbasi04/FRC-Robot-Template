package lib.subsystem.io;

import lib.Utils;

/**
 * Template SubsystemIO class for subsystems that have linear movement
 */
public abstract class LinearSubsystemIO implements ISubsystemIO {
    protected double desiredPosition = 0.0;
    protected double positionTolerance = 0.0;

    protected LinearSubsystemIO(double positionTolerance) {
        this.positionTolerance = positionTolerance;
    }

    /**
     * Sets the target position
     */
    public abstract void setPosition(double units);

    /**
     * Returns the current position
     */
    public abstract double getPosition();

    /**
     * Returns whether the subsystem is at its target
     */
    public boolean atPosition() {
        return atPosition(getDesiredPosition());
    }

    /**
     * Returns whether the subsystem is at the indicated target
     */
    public boolean atPosition(double target) {
        return Utils.isWithin(getPosition(), target, positionTolerance);
    }

    /**
     * Returns the desired position
     */
    public double getDesiredPosition() {
        return this.desiredPosition;
    }

    @Override
    public void halt() {
        setPosition(getPosition());
    }
}