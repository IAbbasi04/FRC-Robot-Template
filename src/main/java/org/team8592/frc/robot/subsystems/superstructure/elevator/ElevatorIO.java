package org.team8592.frc.robot.subsystems.superstructure.elevator;

import org.team8592.lib.hardware.SubsystemIO;

public abstract class ElevatorIO implements SubsystemIO {
    public abstract void setInches(double inches);
    public abstract void setPercentOutput(double percentOutput);
    public abstract double getInches();
}