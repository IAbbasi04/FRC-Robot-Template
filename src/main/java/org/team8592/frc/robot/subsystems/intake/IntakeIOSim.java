package org.team8592.frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

public class IntakeIOSim extends IntakeIO {
    private TalonFXSimState simState;

    private double desiredPercent = 0d;

    public IntakeIOSim(int id, boolean reversed) {
        super(id, reversed);

        simState = new TalonFXSimState(new TalonFX(id));
        simState.setSupplyVoltage(12);
    }

    @Override
    public void setVelocityRPM(double desiredVelocityRPM) {
        this.simState.setRotorVelocity(desiredVelocityRPM / 60d);
    }

    @Override
    public double getVelocityRPM() {
        return this.simState.getMotorVoltage() * 502.1;
    }

    @Override
    public void setSpeedPercentOutput(double percent) {
        this.desiredPercent = percent;
        this.simState.setRotorVelocity(this.simState.getMotorVoltage() * 502.1);
    }

    @Override
    public double getAppliedVoltage() {
        return this.simState.getMotorVoltage();
    }

    @Override
    public double getDesiredPercent() {
        return this.desiredPercent;
    }
}