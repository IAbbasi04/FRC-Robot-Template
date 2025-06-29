package lib.io;

/**
 * Template SubsystemIO class for subsystems that are freely spinning
 */
public abstract class VelocitySubsystemIO implements ISubsystemIO {
    protected double desiredRPM = 0.0;

    /**
     * Set target velocity in RPM
     */
    public abstract void setVelocity(double desiredRPM);

    /**
     * Get current velocity in RPM
     */
    public abstract double getVelocityRPM();

    /**
     * Set target velocity in percent output
     */
    public abstract void setPercentOutput(double desiredPercent);

    /**
     * Get the applied voltage
     */
    public abstract double getOutputVoltage();

    /**
     * Get the desired velocity in RPM
     */
    public double getDesiredRPM() {
        return this.desiredRPM;
    }

    @Override
    public void halt() {
        setVelocity(0d);
    }
}