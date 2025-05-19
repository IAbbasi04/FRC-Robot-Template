package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.Suppliers;

public class SimulatedCommand extends WrapperCommand {
    public SimulatedCommand(Command commandToWrap, double simTime) {
        super(
            Suppliers.IS_SIMULATION.getAsBoolean() ? commandToWrap : commandToWrap.withTimeout(simTime)
        );
    }
}