package lib.subsystem.io;

/**
 * Template SubsystemIO class for subsystems that have rotational movement
 */
public abstract class RotationalSubsystemIO implements ISubsystemIO {
    protected double desiredDegrees = 0.0;
    protected double degreeTolerance = 0.0;

    protected RotationalSubsystemIO(double degreeTolerance) {
        this.degreeTolerance = degreeTolerance;
    }

    /**
     * Sets the target angle
     */
    public abstract void setDegrees(double degrees);

    /**
     * Returns the current angle
     */
    public abstract double getDegrees();

    /**
     * Returns whether the subsystem is at its target angle
     */
    public boolean atAngleDegrees() {
        return atAngleDegrees(getDesiredDegrees());
    }

    /**
     * Returns whether the subsystem is at the indicated target angle
     */
    public boolean atAngleDegrees(double targetDegrees) {
        return Math.abs(getDegrees() - targetDegrees) < degreeTolerance;
    }

    /**
     * Returns the desired angle
     */
    public double getDesiredDegrees() {
        return this.desiredDegrees;
    }

    @Override
    public void halt() {
        setDegrees(getDegrees());
    }
}