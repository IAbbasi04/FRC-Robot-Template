package org.team8592.frc.robot.commands.largecommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import org.team8592.frc.robot.subsystems.SubsystemManager;
// import org.team8592.frc.robot.subsystems.Intake;

public abstract class LargeCommand extends Command {
    protected static SubsystemManager manager;
    public static void addSubsystems(SubsystemManager manager){
        LargeCommand.manager = manager;
    }

    // Require at least one subsystem to be passed in
    public LargeCommand(Subsystem requirement1, Subsystem... moreRequirements){
        addRequirements(requirement1);
        addRequirements(moreRequirements);
    }
} 