package org.team8592.frc.robot.subsystems.intake;

import org.team8592.frc.robot.subsystems.SubsystemIO;

public abstract class IntakeIO implements SubsystemIO {
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
}