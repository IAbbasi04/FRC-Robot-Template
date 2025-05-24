package frc.robot.subsystems.gripper;

import lib.team8592.hardware.motor.*;

public class GripperIOTalonFX extends GripperIO {
    private TalonFXMotor gripperMotor;

    public GripperIOTalonFX(PortConfig config) {
        this.gripperMotor = new TalonFXMotor(config);
    }

    @Override
    public void setDegrees(double degrees) {
        super.desiredDegrees = degrees;
        this.gripperMotor.setPosition(fromDegreesToMotorRotations(degrees));
    }

    @Override
    public double getDegrees() {
        return fromMotorRotationsToDegrees(gripperMotor.getRotations());
    }
}