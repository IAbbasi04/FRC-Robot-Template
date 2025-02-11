package org.team8592.lib.simulation;

public class SimUtils {
    // public static DCMotorSim createSimSparkFlex(double gearing, double momentOfInertia) {
    //     return new DCMotorSim(DCMotor.getNeoVortex(1), gearing, momentOfInertia);
    // }

    protected double addSimFriction(double motorVoltage, double frictionVoltage) {
        if (Math.abs(motorVoltage) < frictionVoltage) {
            motorVoltage = 0.0;
        } else if (motorVoltage > 0.0) {
            motorVoltage -= frictionVoltage;
        } else {
            motorVoltage += frictionVoltage;
        }
        return motorVoltage;
    }
}