package org.team8592.frc.robot.subsystems.intake;

public abstract class IntakeIO {
    protected int id;
    protected boolean reversed;

    public IntakeIO(int id, boolean reversed) {
        this.id = id;
        this.reversed = reversed;
    }

    public abstract void setVelocityRPM(double desiredVelocityRPM);

    public abstract double getVelocityRPM();

    public abstract void setSpeedPercentOutput(double percent);
    
    public abstract double getAppliedVoltage();

    public abstract double getDesiredPercent();
}