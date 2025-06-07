package lib.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

public class PrintCommand extends WrapperCommand {
    public PrintCommand(String message) {
        super(
            new InstantCommand(() -> System.out.println(message))
        );
    }
}