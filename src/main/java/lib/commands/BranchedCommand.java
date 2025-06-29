package lib.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.*;

/**
 * Command that runs a root command, but completes a branch command first if the condition is met
 */
public class BranchedCommand extends WrapperCommand {
    public BranchedCommand(Command root, Command branch, BooleanSupplier branchCondition) {
        super(new ConditionalCommand(
            root, 
            branch.andThen(root), 
            branchCondition
            )
        );
    }
}