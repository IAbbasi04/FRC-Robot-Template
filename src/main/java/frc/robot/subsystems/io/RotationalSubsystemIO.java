package frc.robot.subsystems.io;

public abstract class RotationalSubsystemIO implements ISubsystemIO {
    protected double desiredDegrees = 0d;

    public abstract void setDegrees(double degrees);

    public abstract double getDegrees();

    protected abstract double fromDegreesToMotorRotations(double degrees);

    protected abstract double fromMotorRotationsToDegrees(double rotations);

    protected abstract boolean atAngleDegrees();

    public boolean atAngleDegrees(double degrees, double tolerance) {
        return Math.abs(getDegrees() - degrees) < tolerance;
    }

    public boolean atAngleDegrees(double tolerance) {
        return atAngleDegrees(desiredDegrees, tolerance);
    }

    public double getDesiredDegrees() {
        return desiredDegrees;
    }

    @Override
    public void halt() {
        setDegrees(getDegrees());
    }
}