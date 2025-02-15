package org.team8592.frc.robot.subsystems.intake;

import org.team8592.lib.hardware.motor.talonfx.KrakenX60Motor;

import static org.team8592.frc.robot.Constants.INTAKE.*;

public class IntakeIOKrakenX60 extends IntakeIO {
    private KrakenX60Motor motor;

    public IntakeIOKrakenX60(int id, boolean reversed) {
        super(id, reversed);
        this.motor = new KrakenX60Motor(id, reversed);
        this.motor.withGains(ROLLER_GAINS);
    }

    @Override
    public void setVelocityRPM(double desiredVelocityRPM) {
        motor.setVelocity(desiredVelocityRPM);
    }

    @Override
    public double getVelocityRPM() {
        return motor.getVelocityRPM();
    }

    @Override
    public void setSpeedPercentOutput(double percent) {
        motor.setPercentOutput(percent);
    }

    @Override
    public double getAppliedVoltage() {
        return motor.getAppliedVoltage();
    }

    @Override
    public void updateInputs() {
        // So far nothing
    }
}