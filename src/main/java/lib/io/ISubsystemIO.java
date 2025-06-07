package lib.io;

public interface ISubsystemIO {
    public default void updateInputs() {} // Periodic call that is used to frequently update values within the subsystem
    public default void halt() {} // Prevent the subsystem from moving
}