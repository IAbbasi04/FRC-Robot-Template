package frc.robot.subsystems.gripper;

import lib.team8592.hardware.motor.talonfx.TalonFXMotor;

public class GripperIOReal extends GripperIO {
    private TalonFXMotor gripperMotor;

    public GripperIOReal(int motorID, boolean inverted) {
        this.gripperMotor = new TalonFXMotor(motorID, inverted);
    }

    @Override
    public void setAngle(double degrees) {
        super.desiredDegrees = degrees;
        this.gripperMotor.setPosition(fromDegreesToMotorRotations(degrees));
    }

    @Override
    public double getAngle() {
        return fromMotorRotationsToDegrees(gripperMotor.getRotations());
    }

    @Override
    public void updateInputs() {}
}