package frc.robot.subsystems.roller;

import lib.hardware.motor.*;

public class RollerIOReal extends RollerIO {
    private BaseMotor rollerMotor;

    public RollerIOReal(PortConfig config) {
        this.rollerMotor = MotorFactory.createDefaultTalonFX(config);
        this.rollerMotor.withGains(RollerConstants.ROLLER_GAINS);
    }

    @Override
    public void setVelocity(double desiredRPM) {
        this.desiredRPM = desiredRPM;
        this.rollerMotor.setVelocity(desiredRPM);
    }

    @Override
    public double getVelocityRPM() {
        return this.rollerMotor.getVelocityRPM();
    }

    @Override
    public void setPercentOutput(double desiredPercent) {
        this.rollerMotor.setPercentOutput(desiredPercent);
    }

    @Override
    public double getOutputVoltage() {
        return this.rollerMotor.getAppliedVoltage();
    }
}