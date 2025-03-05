package org.team8592.frc.robot.subsystems;

public interface SubsystemIO {
    public default void updateInputs() {
        // Gives ability to add if needed
        // Mainly necessary for simulation
    }
}