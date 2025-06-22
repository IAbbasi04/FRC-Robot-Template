package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import lib.MatchMode;
import lib.io.ISubsystemIO;
import lib.logging.SmartLogger;
import lib.logging.SubsystemDataMap;

public abstract class Subsystem<E extends ISubsystemIO, V extends Enum<V>> extends SubsystemBase {
    private boolean enabled = false; // TODO - Get Working
    protected E io;
    public SubsystemDataMap<V> data = new SubsystemDataMap<>(); // accesible

    protected SmartLogger logger;
    private Class<V> dataClz;

    protected Subsystem(E io, Class<V> dataClz) {
        this.io = io;
        this.dataClz = dataClz;
        this.logger = new SmartLogger(getName());
        for (V key : dataClz.getEnumConstants()) {
            this.data.set(key, 0d);
        }
    }

    public boolean currentlyCommanded(){
        return getCurrentCommand() != null && getCurrentCommand() != getDefaultCommand();
    }

    public void enableSubsystem(boolean enable) {
        this.enabled = enable;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public V getData() {
        try {
            return (V)dataClz.getDeclaredConstructor().newInstance();
        } catch (Exception e) {
            return null;
        }
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
     * Set the default command of a subsystem (what to run if no other command requiring it is running).
     *
     * @param command to command to set as default
     */
    @Override
    public void setDefaultCommand(Command command) {
        super.setDefaultCommand(command.withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    }

    public void setDefaultCommand(Runnable runnable) {
        this.setDefaultCommand(run(runnable));
    }

    public void removeDefaultCommand() {
        super.removeDefaultCommand();
    }

    public Command getCurrentCommand() {
        if (super.getCurrentCommand() == null) {// No command currently running
            return Commands.none();
        }
        return super.getCurrentCommand();
    }

    public Command getDefaultCommand() {
        if (super.getDefaultCommand() == null) {// No default command
            return Commands.none();
        }
        return super.getDefaultCommand();
    }

    public void periodic() {
        periodicTelemetry();
        periodicOutputs();
        io.updateInputs();
        if (logger != null) {
            logger.log("Actively Commanded", !getCurrentCommand().equals(Commands.none()));
            logger.log("Has Default Command", !getDefaultCommand().equals(Commands.none()));
            logger.log("Active Command", getCurrentCommand().getName());
            logger.log("Default Command", getDefaultCommand().getName());
            for (V value : dataClz.getEnumConstants()) {
                logger.log(value.name(), data.get(value).toString());
            }
        }
    }
    
    public void periodicOutputs() {}

    public abstract void onModeInit(MatchMode mode);

    public abstract void periodicTelemetry();

    public abstract void stop();

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