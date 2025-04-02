package org.team8592.frc.robot.subsystems.superstructure.wrist;

import org.team8592.frc.robot.Constants;
import org.team8592.lib.Utils;
import org.team8592.lib.hardware.SubsystemIO;

public abstract class WristIO implements SubsystemIO {
    protected double targetDegrees = 0d;
    protected double targetVoltage = 0d;

    public abstract void setPercentOutput(double desiredPercent);

    public abstract void setDegrees(double desiredDegrees);

    public abstract double getDegrees();

    public abstract double getVoltage();

    public abstract double getTargetVoltage();

    public abstract double getVelocityRPM();

    public boolean atTargetPosition() {
        return Utils.isWithin(getDegrees(), getTargetDegrees(), Constants.WRIST.WRIST_POSITION_TOLERANCE);
    }

    public double getTargetDegrees() {
        return this.targetDegrees;
    }

    public void trimAngle(double trimDeltaDegrees) {
        this.setDegrees(getDegrees() + trimDeltaDegrees);
    }

    public void haltMovement() {
        setDegrees(getDegrees());
    }
}