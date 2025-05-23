package frc.robot.subsystems.gripper;

import static frc.robot.subsystems.gripper.GripperConstants.*;

import frc.robot.subsystems.io.IHaltableSubsystemIO;
import frc.robot.subsystems.io.SubsystemIO;

public abstract class GripperIO implements SubsystemIO, IHaltableSubsystemIO {
    protected double desiredDegrees = 0d;

    public abstract void setAngle(double degrees);

    public abstract double getAngle();

    public double getDesiredAngle() {
        return desiredDegrees;
    }

    @Override
    public void halt() {
        setAngle(getAngle());
    }

    protected double fromDegreesToMotorRotations(double degrees) {
        return (degrees / 360d) / GEAR_RATIO;
    }

    protected double fromMotorRotationsToDegrees(double rotations) {
        return (rotations * 360d) * GEAR_RATIO;
    }
}