package org.team8592.frc.robot.unittest;

import org.team8592.frc.robot.subsystems.*;
import org.team8592.lib.MatchMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public abstract class SingleSubsystemTest<S extends NewtonSubsystem> extends UnitTest {
    protected S subsystem;
    protected SubsystemManager manager;

    protected SingleSubsystemTest(SubsystemManager manager, S subsystem) {
        this.manager = manager;
        this.subsystem = subsystem;

        timer = new Timer();
        timer.reset();
    }

    @Override
    public Command initTest() {
        return this.subsystem.runOnce(() -> {
            this.subsystem.onInit(MatchMode.TEST);
            this.timer.start();
        });
    }

    @Override
    public Command onFinish() {
        return this.subsystem.getStopCommand();
    }
}