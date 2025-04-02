package org.team8592.frc.robot.subsystems.superstructure.shoulder;

import org.team8592.lib.hardware.SubsystemIO;

public abstract class ShoulderIO implements SubsystemIO {
    public abstract void setDegrees(double degrees);

    public abstract double getDegrees();

    public abstract void setPercentOutput(double percent);
}