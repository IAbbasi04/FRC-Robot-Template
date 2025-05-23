package frc.robot.subsystems.gripper;

import frc.robot.subsystems.io.*;

import static frc.robot.subsystems.gripper.GripperConstants.*;

public abstract class GripperIO extends RotationalSubsystemIO {
    protected double fromDegreesToMotorRotations(double rotations) {
        return (rotations * 360) * GEAR_RATIO;
    }

    protected double fromMotorRotationsToDegrees(double degrees) {
        return (degrees / 360) / GEAR_RATIO;
    }

    @Override
    public boolean atAngleDegrees() {
        return atAngleDegrees(POSITION_TOLERANCE_DEGREES);
    }
}