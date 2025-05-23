package frc.robot.subsystems.wrist;

import lib.team8592.Utils;

import static frc.robot.subsystems.wrist.WristConstants.*;

import frc.robot.subsystems.io.IHaltableSubsystemIO;
import frc.robot.subsystems.io.SubsystemIO;

public abstract class WristIO implements SubsystemIO, IHaltableSubsystemIO {
    protected double desiredDegrees = 0.0;

    public abstract void setAngle(double degrees);

    public abstract double getAngle();

    public boolean atPosition() {
        return Utils.isWithin(getAngle(), desiredDegrees, AT_POSITION_THRESHOLD);
    }

    public double getDesiredAngle() {
        return this.desiredDegrees;
    }

    @Override
    public void halt() {
        setAngle(getAngle());
    }
}