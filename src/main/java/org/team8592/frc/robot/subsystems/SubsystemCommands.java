package org.team8592.frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;

public abstract class SubsystemCommands<S extends NewtonSubsystem<?>> {
    protected S subsystem;

    public SubsystemCommands(S subsystem) {
        this.subsystem = subsystem;
    }

    public Command stopCommand() {
        return subsystem.runOnce(() -> subsystem.stop());
    }
}