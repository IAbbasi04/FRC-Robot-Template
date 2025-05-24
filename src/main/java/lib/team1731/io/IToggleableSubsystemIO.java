package lib.team1731.io;

@Deprecated (forRemoval = false)
public interface IToggleableSubsystemIO extends ISubsystemIO {
    // TODO figure out how to use this
    public void enable(boolean enable);

    public boolean isEnabled();
}