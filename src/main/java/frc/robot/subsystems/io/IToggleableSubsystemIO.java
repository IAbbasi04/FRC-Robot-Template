package frc.robot.subsystems.io;

@Deprecated (forRemoval = false)
public interface IToggleableSubsystemIO extends ISubsystemIO {
    // TODO figure out how to use this
    public void enable(boolean enable);

    public boolean isEnabled();
}