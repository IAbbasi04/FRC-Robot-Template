package org.team8592.frc.robot.subsystems.roller;

import org.team8592.frc.robot.subsystems.SubsystemIO;

public abstract class RollerIO implements SubsystemIO {
    public abstract void setVelocityRPM(double desiredRPM);

    public abstract void setPercentOutput(double desiredPercent);

    public abstract double getVelocityRPM();

    public abstract double getVoltage();

    public abstract double getMaxVelocityRPM();
}
