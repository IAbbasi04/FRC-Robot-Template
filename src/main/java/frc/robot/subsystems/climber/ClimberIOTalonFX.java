package frc.robot.subsystems.climber;

import lib.team8592.hardware.motor.PortConfig;
import lib.team8592.hardware.motor.TalonFXMotor;

public class ClimberIOTalonFX extends ClimberIO {
    private TalonFXMotor climbMotor;

    public ClimberIOTalonFX(PortConfig config) {
        this.climbMotor = new TalonFXMotor(config);
    }

    @Override
    public void setDegrees(double degrees) {
        this.climbMotor.setPosition((degrees / 360) / ClimberConstants.GEAR_RATIO);
    }

    @Override
    public double getDegrees() {
        return (this.climbMotor.getRotations() * 360) * ClimberConstants.GEAR_RATIO;
    }
}