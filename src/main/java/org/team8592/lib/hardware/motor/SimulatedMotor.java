package org.team8592.lib.hardware.motor;

import org.team8592.lib.PIDProfile;

public class SimulatedMotor extends NewtonMotor {

    public SimulatedMotor(int id, boolean reversed, Class<? extends NewtonMotor> cls) {
        super(id, reversed, new MotorConstants(0, 0, 0, 0));
    }

    @Override
    public void setInverted(boolean inverted) {
    }

    @Override
    public void withGains(PIDProfile gains) {
    }

    @Override
    public void setPercentOutput(double percent) {
    }

    @Override
    public void setVoltage(double voltage, int slot) {
    }

    @Override
    public void setVelocity(double desiredRPM, int pidSlot) {
    }

    @Override
    public void setPosition(double desiredRotations, int pidSlot) {
    }

    @Override
    public void setFollowerTo(NewtonMotor master, boolean reversed) {
    }

    @Override
    public void setCurrentLimit(int currentAmps) {
    }

    @Override
    public void setIdleMode(IdleMode idleMode) {
    }

    @Override
    public double getVelocityRPM() {
        return 0d;
    }

    @Override
    public double getRotations() {
        return 0d;
    }

    @Override
    public double getAppliedVoltage() {
        return 0d;
    }

    @Override
    public void resetEncoderPosition(double rotations) {
    }

    @Override
    public void setSoftLimits(double min, double max) {
    }

    @Override
    public void configureMotionProfile(double maxVelocity, double maxAcceleration) {
    }

    @Override
    public double getVoltage() {
        return 0d;
    }
    
}