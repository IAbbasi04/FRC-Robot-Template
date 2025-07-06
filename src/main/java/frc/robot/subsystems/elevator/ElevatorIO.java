package frc.robot.subsystems.elevator;

import lib.subsystem.io.LinearSubsystemIO;

public abstract class ElevatorIO extends LinearSubsystemIO {
    public ElevatorIO() {
        super(0.1); // Inches
    }
}