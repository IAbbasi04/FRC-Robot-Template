package frc.robot.subsystems.wrist;

import lib.team8592.hardware.motor.talonfx.TalonFXMotor;

public class WristIOReal extends WristIO {
    private TalonFXMotor wristMotor;

    public WristIOReal(int id, boolean reversed) {
        this.wristMotor = new TalonFXMotor(id, reversed);
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