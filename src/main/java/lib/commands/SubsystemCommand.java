package lib.commands;

import edu.wpi.first.wpilibj2.command.*;
import lib.subsystem.BaseSubsystem;

/**
 * Wrapper class that adds a few convenience methods for commands within a subsystem
 */
public class SubsystemCommand extends WrapperCommand {
    public SubsystemCommand(Command command, String name, BaseSubsystem<?, ?> subsystem) {
        super(command.onlyIf(() -> subsystem.isEnabled()));
        super.m_command.setName(name);
    }

    public SubsystemCommand(Command command, BaseSubsystem<?, ?> subsystem) {
        super(command.onlyIf(() -> subsystem.isEnabled()));
    }
}