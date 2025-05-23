package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import frc.robot.subsystems.io.LinearSubsystemIO;

public abstract class ElevatorIO extends LinearSubsystemIO {
    protected double getMotorRotationsFromInches(double inches) {
        return (inches / (Math.PI * DRUM_RADIUS_METERS)) / GEAR_RATIO;
    }

    protected double getInchesFromMotorRotations(double rotations) {
        return (rotations * DRUM_RADIUS_METERS * Math.PI) * GEAR_RATIO;
    }

    @Override
    public boolean atPosition() {
        return atPosition(desiredPosition, ElevatorConstants.POSITION_TOLERANCE_INCHES);
    }
}