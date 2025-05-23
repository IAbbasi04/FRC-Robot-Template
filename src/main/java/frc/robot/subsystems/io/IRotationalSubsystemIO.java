package frc.robot.subsystems.io;

public interface IRotationalSubsystemIO {
    public void setAngle(double degrees);

    public double getAngle();

    public double getDesiredAngle();
    
    public boolean atPosition();

    public default boolean atPosition(double position, double tolerance) {
        return Math.abs(getAngle() - position) < tolerance;
    }
}