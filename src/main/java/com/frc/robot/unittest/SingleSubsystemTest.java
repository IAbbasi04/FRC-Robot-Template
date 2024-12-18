package com.frc.robot.unittest;

import com.frc.robot.subsystems.NewtonSubsystem;
import com.frc.robot.subsystems.SubsystemManager;
import com.lib.team8592.MatchMode;

import edu.wpi.first.wpilibj.Timer;

public abstract class SingleSubsystemTest<S extends NewtonSubsystem> extends UnitTest {
    protected Timer timer = new Timer();
    private S subsystem;
    protected SubsystemManager manager;

    protected SingleSubsystemTest(SubsystemManager manager, S subsystem) {
        this.manager = manager;
        this.subsystem = subsystem;

        timer.stop();
        timer.reset();
    }

    @Override
    public void initialize() {
        this.subsystem.onInit(MatchMode.TEST);
        this.initTest();
    }

    @Override
    public void run() {
        this.timer.start();
        this.subsystem.periodicLogs();
        this.updateTest();
    }

    @Override
    public void shutdown() {
        this.subsystem.stop();
    }

    public abstract void initTest();

    public abstract void updateTest();
}