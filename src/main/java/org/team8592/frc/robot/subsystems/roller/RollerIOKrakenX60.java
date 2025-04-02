package org.team8592.frc.robot.subsystems.roller;

import org.team8592.lib.PIDProfile;
import org.team8592.lib.hardware.motor.talonfx.*;

public class RollerIOKrakenX60 extends RollerIO {
    private KrakenX60Motor rollerMotor;
    private PIDProfile velocityGains = new PIDProfile()
        .setP(1E-3)
        .setV(1E-2)
    ;

    public RollerIOKrakenX60(int CANId, boolean reversed) {
        this.rollerMotor = new KrakenX60Motor(CANId, reversed);
        this.rollerMotor.withGains(velocityGains);
    }

    public RollerIOKrakenX60(int CANId) {
        this(CANId, false);
    }

    @Override
    public void setVelocityRPM(double desiredRPM) {
        this.rollerMotor.setVelocity(desiredRPM);
    }

    @Override
    public void setPercentOutput(double desiredPercent) {
        this.rollerMotor.setPercentOutput(desiredPercent);
    }

    @Override
    public double getVelocityRPM() {
        return this.rollerMotor.getVelocityRPM();
    }

    @Override
    public double getVoltage() {
        return this.rollerMotor.getMotorVoltage();
    }

    @Override
    public double getMaxVelocityRPM() {
        return this.rollerMotor.getMaxFreeVelocity();
    }
    
}