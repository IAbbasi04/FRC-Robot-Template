package frc.robot.commands;

import java.util.Set;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

public class FluidCommand extends WrapperCommand {
    public FluidCommand(Command commandOne, Command commandTwo, BooleanSupplier firstEndingCondition, BooleanSupplier secondEndingCondition, Subsystem[] requirements) {
        super(new DeferredCommand(
            () -> {
                if (firstEndingCondition.getAsBoolean()) {
                    return commandTwo;
                }
                return commandOne;
            },
            Set.of(requirements))
            .until(secondEndingCondition)
        );
    }
}