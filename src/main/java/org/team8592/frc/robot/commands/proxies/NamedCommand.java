package org.team8592.frc.robot.commands.proxies;

import edu.wpi.first.wpilibj2.command.*;

/**
 * Best way I can think of to do this ... probably a better way
 */
public class NamedCommand extends Command {
    private final Command command;

    public NamedCommand(String name, Command command) {
        this.command = command;
        this.command.setName(name);
        this.setName(name);
        this.addRequirements(command.getRequirements());
    }

    @Override
    public void initialize() {
        command.initialize();
    }

    @Override
    public void execute() {
        command.execute();
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);
    }
}