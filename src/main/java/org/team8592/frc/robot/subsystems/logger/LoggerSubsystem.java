package org.team8592.frc.robot.subsystems.logger;

import org.team8592.frc.robot.subsystems.NewtonSubsystem;
import org.team8592.frc.robot.subsystems.SubsystemCommands;
import org.team8592.lib.MatchMode;

public class LoggerSubsystem extends NewtonSubsystem<SubsystemCommands<?>> {
    public LoggerSubsystem(boolean logToShuffleboard) {
        super(logToShuffleboard);

        
    }

    @Override
    public void periodic() {

    }

    @Override
    public void onModeInit(MatchMode mode) {

    }

    @Override
    public void periodicTelemetry() {

    }

    @Override
    public void stop() {

    }
}