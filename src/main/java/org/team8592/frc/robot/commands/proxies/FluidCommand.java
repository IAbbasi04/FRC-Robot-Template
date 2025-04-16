package org.team8592.frc.robot.commands.proxies;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

public class FluidCommand extends WrapperCommand {
    /**
     * Command to allow seamless transition between two commands 
     * - i.e. go to one position then go to another without a pause between both
     */
    public FluidCommand(Command firstCommand, Command secondCommand, BooleanSupplier transitionCondition) {
        super(firstCommand.until(transitionCondition).andThen(secondCommand));
    }
}
