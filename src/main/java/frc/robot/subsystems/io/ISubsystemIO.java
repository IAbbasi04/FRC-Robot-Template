package frc.robot.subsystems.io;

public interface ISubsystemIO {
    public void updateInputs(); // Periodic call that is used to frequently update values within the subsystem
    public void halt(); // Stops the subsystem
}