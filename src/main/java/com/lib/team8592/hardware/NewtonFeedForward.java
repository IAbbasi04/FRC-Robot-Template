package com.lib.team8592.hardware;

public class NewtonFeedForward {
    public double kV = 0d;
    public double kA = 0d;
    public double kS = 0d;
    public double kG = 0d;

    private DoubleSupplier angle = () -> 0d;

    private double lastVelocity = 0d;

    public NewtonFeedForward() {}

    public NewtonFeedForward(double kV, double kA, double kS) {
        this.kV = kV;
        this.kA = kA;
        this.kS = kS;
    }

    public NewtonFeedForward withKG(double kG, DoubleSupplier angle) {
        this.angle = angle;
    }

    public double get(double velocity, double acceleration) {
        this.lastVelocity = velocity;
        return this.kG + this.kS + this.kV * velocity + this.kA * acceleration;
    }

    public double get(double lastVelocity, double velocity, double dt) {
        this.lastVelocity = lastVelocity;
        return this.get(velocity, accel);
    }

    public double get(double velocity, double dt) {
        double accel = (velocity - lastVelocity) / dt;
        return this.get(velocity, accel);
    }
}