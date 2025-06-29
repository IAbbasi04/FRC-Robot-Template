package lib.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.*;

/**
 * Wrapper command that only runs the indicated command once the given condition is true
 */
public class WaitUntilCommand extends WrapperCommand{
    public WaitUntilCommand(Command command, BooleanSupplier startCondition){
        super(new WaitCommand(Double.POSITIVE_INFINITY).until(startCondition).andThen(command));
    }

    public WaitUntilCommand(BooleanSupplier startCondition) {
        this(Commands.none(), startCondition);
    }

    public WaitUntilCommand(Command command, double waitTime){
        super(new WaitCommand(waitTime).andThen(command));
    }
}
