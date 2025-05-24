package frc.robot.subsystems.roller;

import lib.team8592.PIDProfile;
import lib.team8592.hardware.motor.PortConfig;
import lib.team8592.hardware.motor.TalonFXMotor;

public class RollerIOTalonFX extends RollerIO {
    private TalonFXMotor motor;

    public RollerIOTalonFX(PortConfig config) {
        this.motor = new TalonFXMotor(config);
        this.motor.withGains(new PIDProfile().setP(1E-3).setV(1E-1).setMaxVelocity(5000).setMaxAcceleration(10000).setSlot(0));
    }

    @Override
    public void setVelocity(double desiredRPM) {
        this.motor.setVelocity(desiredRPM);
    }

    @Override
    public double getVelocityRPM() {
        return this.motor.getVelocityRPM();
    }

    @Override
    public void setPercentOutput(double desiredPercent) {
        this.motor.setPercentOutput(desiredPercent);
    }

    @Override
    public double getOutputVoltage() {
        return this.motor.getAppliedVoltage();
    }
}