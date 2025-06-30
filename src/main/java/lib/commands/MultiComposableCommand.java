package lib.commands;

import edu.wpi.first.wpilibj2.command.*;

/**
 * Proxy command that does not register its command as a
 * composed command.
 *
 * @param command the command to proxy
 *
 * @apiNote This command will require the same subsystems
 * as the passed-in command.
 *
 * @apiNote This command wraps the passed-in command. If
 * this command is cancelled, the passed-in command will
 * be cancelled too.
 */
public class MultiComposableCommand extends FunctionalCommand{
    public MultiComposableCommand(Command command){
        super(
            () -> command.initialize(),
            () -> command.execute(),
            (interrupted) -> command.end(interrupted),
            () -> command.isFinished(),
            command.getRequirements().toArray(new Subsystem[0])
        );
    }
}
