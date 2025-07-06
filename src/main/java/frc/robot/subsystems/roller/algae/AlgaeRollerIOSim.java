package frc.robot.subsystems.roller.algae;

import lib.hardware.motor.MotorConstants;

public class AlgaeRollerIOSim extends AlgaeRollerIO {
    @Override
    public void setVelocity(double desiredRPM) {
        this.desiredRPM = desiredRPM;
    }

    @Override
    public double getVelocityRPM() {
        return this.desiredRPM;
    }

    @Override
    public void setPercentOutput(double desiredPercent) {
        this.desiredRPM = desiredPercent * MotorConstants.KRAKEN_X60.MAX_VELOCITY_RPM;
    }

    @Override
    public double getOutputVoltage() {
        return this.desiredRPM / MotorConstants.KRAKEN_X60.MOTOR_KV;
    }
}