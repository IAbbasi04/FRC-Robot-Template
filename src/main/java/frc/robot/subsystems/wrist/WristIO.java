package frc.robot.subsystems.wrist;

import lib.team1731.io.RotationalSubsystemIO;

public abstract class WristIO extends RotationalSubsystemIO {
    protected WristIO() {
        super(WristConstants.AT_POSITION_THRESHOLD);
    }

    protected double fromDegreesToMotorRotations(double degrees) {
        return (degrees / 360d) / WristConstants.GEAR_RATIO;
    }

    protected double fromMotorRotationsToDegrees(double rotations) {
        return (rotations * 360d) * WristConstants.GEAR_RATIO;
    }

    @Override
    public void halt() {
        this.setDegrees(getDegrees());
    }
}