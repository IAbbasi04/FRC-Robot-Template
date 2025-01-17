package org.team8592.frc.robot.commands.proxies;

import org.team8592.frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.*;

public abstract class NewtonWrapperCommand extends WrapperCommand {
    protected static SubsystemManager manager;

    public NewtonWrapperCommand(Command command){
        super(command);
        setName(getClass().getSimpleName());
    }

    public static void initialize(SubsystemManager manager) {
        NewtonWrapperCommand.manager = manager;
    }

    public static Command stopSubsystems(NewtonSubsystem... toStop){
        Command result = Commands.none();
        for(NewtonSubsystem s:toStop){
            result = result.alongWith(s.getStopCommand());
        }
        return result;
    }
}
