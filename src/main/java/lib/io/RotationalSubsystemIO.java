package lib.io;

public abstract class RotationalSubsystemIO implements ISubsystemIO {
    protected double desiredDegrees = 0.0;
    protected double degreeTolerance = 0.0;

    protected RotationalSubsystemIO(double degreeTolerance) {
        this.degreeTolerance = degreeTolerance;
    }

    public abstract void setDegrees(double degrees);

    public abstract double getDegrees();

    public boolean atAngleDegrees() {
        return atAngleDegrees(getDesiredDegrees());
    }

    public boolean atAngleDegrees(double targetDegrees) {
        return Math.abs(getDegrees() - targetDegrees) < degreeTolerance;
    }

    public double getDesiredDegrees() {
        return this.desiredDegrees;
    }

    @Override
    public void halt() {
        setDegrees(getDegrees());
    }
}