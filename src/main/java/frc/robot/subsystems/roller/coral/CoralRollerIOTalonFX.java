package frc.robot.subsystems.roller.coral;

import lib.hardware.motor.*;
import lib.hardware.motor.ctre.TalonFXMotor;

public class CoralRollerIOTalonFX extends CoralRollerIO {
    private TalonFXMotor intakeMotor;

    public CoralRollerIOTalonFX(PortConfig config) {
        intakeMotor = new TalonFXMotor(config);
    }

    @Override
    public void setVelocity(double desiredRPM) {
        this.desiredRPM = desiredRPM;
        intakeMotor.setVelocity(desiredRPM);
    }

    @Override
    public double getVelocityRPM() {
        return intakeMotor.getVelocityRPM();
    }

    @Override
    public void setPercentOutput(double desiredPercent) {
        this.desiredRPM = desiredPercent * MotorConstants.KRAKEN_X60.MAX_VELOCITY_RPM;
        intakeMotor.setPercentOutput(desiredPercent);
    }

    @Override
    public double getOutputVoltage() {
        return intakeMotor.getAppliedVoltage();
    }
    
}
