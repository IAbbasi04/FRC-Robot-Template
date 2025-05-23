package frc.robot.subsystems.io;

public abstract class VelocitySubsystemIO implements ISubsystemIO {
    protected double desiredRPM = 0d;

    public abstract void setVelocityRPM(double desiredRPM);

    public abstract double getVelocityRPM();

    public abstract void setPercentOutput(double desiredPercent);

    public abstract double getOutputVoltage();

    public double getDesiredRPM() {
        return desiredRPM;
    }
}