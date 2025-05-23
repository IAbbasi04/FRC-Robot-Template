package frc.robot.subsystems.io;

public interface IVelocitySubsystemIO extends ISubsystemIO {
    public void setVelocity(double desiredRPM);

    public double getVelocityRPM();

    public void setPercentOutput(double desiredPercent);

    public double getOutputVoltage();
}