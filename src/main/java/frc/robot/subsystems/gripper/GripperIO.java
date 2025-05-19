package frc.robot.subsystems.gripper;

import frc.robot.subsystems.SubsystemIO;

import frc.robot.subsystems.IHaltableSubsystemIO;

public abstract class GripperIO implements SubsystemIO, IHaltableSubsystemIO {
    protected double desiredDegrees = 0d;

    public abstract void setAngle(double degrees);

    public abstract double getAngle();

    public double getDesiredAngle() {
        return desiredDegrees;
    }
}