package frc.robot.subsystems.gripper;

import lib.team1731.io.RotationalSubsystemIO;

public abstract class GripperIO extends RotationalSubsystemIO {
    protected GripperIO() {
        super(GripperConstants.POSITION_TOLERANCE_DEGREES);
    }

    protected double fromDegreesToMotorRotations(double rotations) {
        return (rotations * 360) * GripperConstants.GEAR_RATIO;
    }

    protected double fromMotorRotationsToDegrees(double degrees) {
        return (degrees / 360) / GripperConstants.GEAR_RATIO;
    }
}