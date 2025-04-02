package org.team8592.frc.robot.subsystems.superstructure.shoulder;

import org.team8592.lib.hardware.motor.NewtonMotor;
import org.team8592.lib.hardware.motor.talonfx.KrakenX60Motor;

public class ShoulderIOKrakenX60 extends ShoulderIO {
    private NewtonMotor shoulderMotor;

    public ShoulderIOKrakenX60(int CANId, boolean reversed) {
        this.shoulderMotor = new KrakenX60Motor(CANId, reversed);
    }

    public ShoulderIOKrakenX60(int CANId) {
        this(CANId, false);
    }

    @Override
    public void setDegrees(double degrees) {
        this.shoulderMotor.setPosition(degrees);
    }

    @Override
    public double getDegrees() {
        return this.shoulderMotor.getRotations();
    }

    @Override
    public void setPercentOutput(double percent) {
        this.shoulderMotor.setPercentOutput(percent);
    }
}