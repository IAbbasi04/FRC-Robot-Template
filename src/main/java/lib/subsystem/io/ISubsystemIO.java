package lib.subsystem.io;

/**
 * Interface that tags a class as an IO class used by subsystems
 */
public interface ISubsystemIO {
    /**
     * Periodic call that is used to frequently update values within the subsystem
     */
    public default void updateInputs() {}

    /**
     * Prevent the subsystem from moving
     */
    public default void halt() {} 
}