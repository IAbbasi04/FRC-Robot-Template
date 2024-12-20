package com.frc.robot.unittest;

import java.util.List;

import com.frc.robot.subsystems.SubsystemManager;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

public class UnitTestScheduler {
    private Command unitTestCommand;

    public UnitTestScheduler(SubsystemManager manager) {
        UnitTestSequence testSequence = new UnitTestSequence(List.of(
            // Add all unit tests you want to run in sequence below
        ));

        this.unitTestCommand = testSequence.withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
        this.unitTestCommand.addRequirements(manager.getAllSubsystemsAsArray());
    }

    public Command getUnitTestCommand() {
        return this.unitTestCommand;
    }
}