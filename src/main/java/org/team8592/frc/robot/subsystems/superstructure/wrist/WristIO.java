package org.team8592.frc.robot.subsystems.superstructure.wrist;

import org.team8592.lib.hardware.SubsystemIO;

public abstract class WristIO implements SubsystemIO {
    public abstract void setPercentOutput(double desiredPercent);

    public abstract void setDegrees(double degrees);

    public abstract double getDegrees();

    public void trimAngle(double trimDeltaDegrees) {
        this.setDegrees(getDegrees() + trimDeltaDegrees);
    }
}