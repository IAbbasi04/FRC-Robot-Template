package org.team8592.frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

import org.team8592.lib.logging.SmartLogger;
import org.team8592.lib.MatchMode;

public abstract class NewtonSubsystem extends SubsystemBase {
    private boolean enabled = false; // TODO - Get Working

    protected SmartLogger logger;

    protected NewtonSubsystem() {
        this.logger = new SmartLogger(getName());
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
    
    public void periodicOutputs() {}

    public abstract void onModeInit(MatchMode mode);

    public abstract void periodicTelemetry();

    public abstract void stop();
}