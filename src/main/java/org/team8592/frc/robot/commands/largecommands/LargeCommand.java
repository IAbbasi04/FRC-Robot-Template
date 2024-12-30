package org.team8592.frc.robot.commands.largecommands;

import org.team8592.frc.robot.subsystems.NewtonSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public abstract class LargeCommand extends Command {
    // Require at least one subsystem to be passed in
    public LargeCommand(NewtonSubsystem requirement1, NewtonSubsystem... moreRequirements){
        addRequirements(requirement1);
        addRequirements(moreRequirements);
    }
}