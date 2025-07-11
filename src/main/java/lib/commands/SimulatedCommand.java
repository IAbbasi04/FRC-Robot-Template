package lib.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.Robot;

/**
 * Wrapper command that would add a timeout to the indicated command if in simulation
 */
public class SimulatedCommand extends WrapperCommand {
    public SimulatedCommand(Command commandToWrap, double simTime) {
        super(
            Robot.isSimulation() ? commandToWrap : commandToWrap.withTimeout(simTime)
        );
    }
}