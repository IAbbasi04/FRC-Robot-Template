package com.frc.robot.subsystems;

import com.lib.team8592.MatchMode;

public class IntakeSubsystem extends NewtonSubsystem {
    private double desiredPercentOutput = 0d;
    
    protected IntakeSubsystem(boolean logToShuffleboard) {
        super(logToShuffleboard);
    }

    @Override
    public void onInit(MatchMode mode) {}

    @Override
    public void periodicLogs() {
        logger.log("Desired Percent Output", this.desiredPercentOutput);
    }

    @Override
    public void stop() {}
}