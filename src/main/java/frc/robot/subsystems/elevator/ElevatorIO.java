package frc.robot.subsystems.elevator;

import lib.team1731.io.LinearSubsystemIO;

public abstract class ElevatorIO extends LinearSubsystemIO {
    protected ElevatorIO() {
        super(ElevatorConstants.POSITION_TOLERANCE_INCHES);
    }

    protected double getMotorRotationsFromInches(double inches) {
        return (inches / (Math.PI * ElevatorConstants.DRUM_RADIUS_METERS)) / ElevatorConstants.GEAR_RATIO;
    }

    protected double getInchesFromMotorRotations(double rotations) {
        return (rotations * ElevatorConstants.DRUM_RADIUS_METERS * Math.PI) * ElevatorConstants.GEAR_RATIO;
    }
}