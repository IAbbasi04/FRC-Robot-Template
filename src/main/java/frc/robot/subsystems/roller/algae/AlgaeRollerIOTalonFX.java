package frc.robot.subsystems.roller.algae;

import lib.hardware.motor.MotorConstants;
import lib.hardware.motor.PortConfig;
import lib.hardware.motor.ctre.TalonFXMotor;

public class AlgaeRollerIOTalonFX extends AlgaeRollerIO {
    private TalonFXMotor intakeMotor;

    public AlgaeRollerIOTalonFX(PortConfig config) {
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
