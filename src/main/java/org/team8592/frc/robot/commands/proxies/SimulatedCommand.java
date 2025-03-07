package org.team8592.frc.robot.commands.proxies;

import org.team8592.frc.robot.Suppliers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

public class SimulatedCommand extends WrapperCommand {
    public SimulatedCommand(Command commandToWrap, double simTime) {
        super(
            Suppliers.IS_RED_ALLIANCE.getAsBoolean() ? commandToWrap : commandToWrap.withTimeout(simTime)
        );
    }
}