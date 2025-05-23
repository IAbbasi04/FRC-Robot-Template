package frc.robot.subsystems.io;

public interface IVelocitySubsystemIO {
    public void setVelocity(double desiredRPM);

    public double getVelocityRPM();

    public void setPercentOutput(double desiredPercent);
}
