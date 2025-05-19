package frc.robot.subsystems.elevator;

import frc.robot.subsystems.SubsystemIO;
import lib.team8592.Utils;

public abstract class ElevatorIO implements SubsystemIO {
    protected double desiredInches = 0d;

    public abstract void setInches(double inches);

    public abstract double getCurrentInches();

    public abstract void halt();

    public double getDesiredInches() {
        return desiredInches;
    }

    public boolean atPosition() {
        return Utils.isWithin(getCurrentInches(), desiredInches, ElevatorConstants.POSITION_TOLERANCE_INCHES);
    }
}