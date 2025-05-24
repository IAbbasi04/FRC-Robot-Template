package frc.robot.subsystems.wrist;

import lib.team8592.hardware.motor.PortConfig;
import lib.team8592.hardware.motor.TalonFXMotor;

public class WristIOTalonFX extends WristIO {
    private TalonFXMotor wristMotor;

    public WristIOTalonFX(PortConfig config) {
        this.wristMotor = new TalonFXMotor(config);
    }

    @Override
    public void setDegrees(double degrees) {
        this.desiredDegrees = degrees;
        wristMotor.setPosition(fromDegreesToMotorRotations(degrees));
    }

    @Override
    public double getDegrees() {
        return fromMotorRotationsToDegrees(wristMotor.getRotations());
    }

    @Override
    public void updateInputs() {}
}