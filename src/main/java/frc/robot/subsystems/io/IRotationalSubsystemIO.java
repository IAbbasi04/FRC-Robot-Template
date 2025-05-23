package frc.robot.subsystems.io;

public interface IRotationalSubsystemIO {
    public void setDegrees(double degrees);

    public double getDegrees();

    public double getDesiredDegrees();
    
    public boolean atAngleDegrees();

    public default boolean atAngleDegrees(double degrees, double tolerance) {
        return Math.abs(getDegrees() - degrees) < tolerance;
    }
}