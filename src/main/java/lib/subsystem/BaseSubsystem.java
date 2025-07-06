package lib.subsystem;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import lib.MatchMode;
import lib.commands.SubsystemCommand;
import lib.logging.SmartLogger;

/**
 * Template class for the robot's subsystems
 */
public abstract class BaseSubsystem extends SubsystemBase {
    private boolean enabled = false; // TODO - Get Working

    protected SmartLogger logger;

    protected BaseSubsystem() {
        this.logger = new SmartLogger(getName());
    }

    /**
     * Runs then initialize method of a command.
     * 
     * Useful for turning a command into a runnable void
     */
    public void doOnce(Command command) {
        command.initialize();
    }

    /**
     * Whether there is a command actively running on this subsystem
     */
    public boolean isCurrentlyCommanded(){
        return getCurrentCommand() != null && getCurrentCommand() != getDefaultCommand();
    }

    /**
     * Enables or disables the subsystem
     */
    public void enableSubsystem(boolean enable) {
        this.enabled = enable;
    }

    /**
     * Returns whether the subsystem is enabled
     */
    public boolean isEnabled() {
        return enabled;
    }

    /**
     * Creates a stop command for this subsystem
     */
    public Command getStopCommand() {
        return this.runOnce(() -> {stop();});
    }

    /**
     * Sets the subsystem's stop method as the default command
     */
    public void setStopAsDefaultCommand() {
        this.setDefaultCommand(getStopCommand());
    }
    
    /**
     * Set the default command of a subsystem (what to run if no other command requiring it is running)
     */
    @Override
    public void setDefaultCommand(Command command) {
        super.setDefaultCommand(command.withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    }

    /**
     * Set the default command of a subsystem (what to run if no other command requiring it is running)
     *
     * @param command to command to set as default
     */
    public void setDefaultCommand(Runnable runnable) {
        this.setDefaultCommand(run(runnable));
    }

    /**
     * Removes the current default command
     */
    public void removeDefaultCommand() {
        super.removeDefaultCommand();
    }

    /**
     * Returns the current command that is actively running on this subsystem
     */
    public Command getCurrentCommand() {
        if (super.getCurrentCommand() == null) {// No command currently running
            return Commands.none();
        }
        return super.getCurrentCommand();
    }

    /**
     * Returns the default command for this subsystem
     */
    public Command getDefaultCommand() {
        if (super.getDefaultCommand() == null) {// No default command
            return Commands.none();
        }
        return super.getDefaultCommand();
    }

    @Override
    public void periodic() {
        periodicTelemetry();
        periodicOutputs();
        if (logger != null) {
            logger.log("Actively Commanded", !getCurrentCommand().equals(Commands.none()));
            logger.log("Has Default Command", !getDefaultCommand().equals(Commands.none()));
            logger.log("Active Command", getCurrentCommand().getName());
            logger.log("Default Command", getDefaultCommand().getName());
        }
    }
    
    /**
     * Optional method to set the outputs of the subsystem periodically
     */
    public void periodicOutputs() {}

    /**
     * Called upon the start of the indicated match mode
     */
    public abstract void onModeInit(MatchMode mode);

    /**
     * Periodic method to update telemetry and data
     */
    public abstract void periodicTelemetry();

    /**
     * Stops the subsystem
     */
    public abstract void stop();

    public SubsystemCommand createSubsystemCommand(String name, Command command) {
        return new SubsystemCommand(command, name, this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Subsystem");

        builder.addBooleanProperty(".hasDefault", () -> getDefaultCommand() != null, null);
        builder.addStringProperty(
            ".default",
            () -> getDefaultCommand() != null ? getDefaultCommand().getName() : "none",
            null);
        builder.addBooleanProperty(".currentlyCommanded", () -> getCurrentCommand() != null, null);
        builder.addStringProperty(
            ".currentCommand",
            () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "none",
            null);
    }
}