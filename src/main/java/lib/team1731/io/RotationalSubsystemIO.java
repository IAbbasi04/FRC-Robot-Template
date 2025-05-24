package lib.team1731.io;

public abstract class RotationalSubsystemIO implements ISubsystemIO {
    protected double desiredDegrees = 0.0;

    public abstract void setDegrees(double degrees);

    public abstract double getDegrees();

    public abstract boolean atAngleDegrees();

    public abstract boolean atAngleDegrees(double targetDegrees);

    public double getDesiredDegrees() {
        return this.desiredDegrees;
    }

    protected boolean atAngleDegrees(double degrees, double tolerance) {
        return Math.abs(getDegrees() - degrees) < tolerance;
    }
}