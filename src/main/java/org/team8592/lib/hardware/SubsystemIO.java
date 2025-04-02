package org.team8592.lib.hardware;

public interface SubsystemIO {
    public default void updateInputs() {
        // Gives ability to add if needed
        // Mainly necessary for simulation
    } 
}