package frc.robot.subsystems.wrist;

import frc.robot.subsystems.io.*;

public abstract class WristIO extends RotationalSubsystemIO {
    @Override
    protected double fromDegreesToMotorRotations(double degrees) {
        return (degrees / 360d) / WristConstants.GEAR_RATIO;
    }

    @Override
    protected double fromMotorRotationsToDegrees(double rotations) {
        return rotations * WristConstants.GEAR_RATIO * 360d;
    }

    @Override
    public boolean atAngleDegrees() {
        return atAngleDegrees(WristConstants.AT_POSITION_THRESHOLD);
    }
}