package org.team8592.frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static org.team8592.frc.robot.Constants.INTAKE.*;

import org.team8592.frc.robot.Robot;

public class IntakeIOSim extends IntakeIO {
    private DCMotorSim motorSim;

    public IntakeIOSim(int id, boolean reversed) {
        super(id, reversed);

        this.motorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(ROLLER_GAINS.kV, ROLLER_GAINS.kA), 
            DCMotor.getKrakenX60(1)
        );
    }

    @Override
    public void setVelocityRPM(double desiredVelocityRPM) {
        this.motorSim.setAngularVelocity(desiredVelocityRPM * 2 * Math.PI / 60d);
    }

    @Override
    public double getVelocityRPM() {
        return this.motorSim.getAngularVelocity().baseUnitMagnitude() * 60d / (2 * Math.PI);
    }

    @Override
    public void setSpeedPercentOutput(double percent) {
        this.motorSim.setInput(percent * 12);
    }

    @Override
    public double getAppliedVoltage() {
        return this.motorSim.getInputVoltage();
    }

    @Override
    public void updateInputs() {
        this.motorSim.update(Robot.CLOCK.dt());
    }
}