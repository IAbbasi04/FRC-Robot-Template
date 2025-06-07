package lib.io;

public abstract class VelocitySubsystemIO implements ISubsystemIO {
    protected double desiredRPM = 0.0;

    public abstract void setVelocity(double desiredRPM);

    public abstract double getVelocityRPM();

    public abstract void setPercentOutput(double desiredPercent);

    public abstract double getOutputVoltage();

    public double getDesiredRPM() {
        return this.desiredRPM;
    }

    @Override
    public void halt() {
        this.setVelocity(0d);
    }
}