package frc.robot.subsystems.roller;

import lib.hardware.motor.PortConfig;
import lib.hardware.motor.ctre.TalonFXMotor;
import lib.hardware.motor.rev.MotorFactory;

public class RollerIOReal extends RollerIO {
    private TalonFXMotor rollerMotor;

    public RollerIOReal(PortConfig config) {
        this.rollerMotor = MotorFactory.createDefaultTalonFX(config);
    }

    @Override
    public void setVelocity(double desiredRPM) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setVelocity'");
    }

    @Override
    public double getVelocityRPM() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getVelocityRPM'");
    }

    @Override
    public void setPercentOutput(double desiredPercent) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPercentOutput'");
    }

    @Override
    public double getOutputVoltage() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getOutputVoltage'");
    }
    
}