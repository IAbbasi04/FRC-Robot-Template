package org.team8592.frc.robot.subsystems.roller;

import org.team8592.frc.robot.Robot;
import org.team8592.lib.PIDProfile;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class RollerIOSim extends RollerIO {
    private PIDProfile velocityGains = new PIDProfile()
        .setP(1d)
        .setV(5E-4)
    ;
    
    private DCMotor rollerMotor = DCMotor.getKrakenX60(1);

    private DCMotorSim rollerMotorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            velocityGains.kV, 
            0.0005
        ),
        rollerMotor,
        new double[] {}
    );

    private PIDController velocityCtrl = velocityGains.toPIDController();

    private final double KRAKEN_KV = 502.1;

    @Override
    public void setVelocityRPM(double desiredRPM) {
        double desiredVoltage = velocityCtrl.calculate(getVelocityRPM(), desiredRPM);
        this.rollerMotorSim.setInput(desiredVoltage);
    }

    @Override
    public void setPercentOutput(double desiredPercent) {
        double desiredVoltage = velocityCtrl.calculate(getVoltage(), desiredPercent * 12d);
        this.rollerMotorSim.setInput(desiredVoltage);
    }

    @Override
    public double getVelocityRPM() {
        return this.rollerMotorSim.getAngularVelocityRPM();
    }

    @Override
    public double getVoltage() {
        return this.rollerMotorSim.getAngularVelocityRPM() / KRAKEN_KV;
    }

    @Override
    public void updateInputs() {
        this.rollerMotorSim.update(Robot.CLOCK.dt());
    }

    @Override
    public double getMaxVelocityRPM() {
        return 6000d;
    }
}