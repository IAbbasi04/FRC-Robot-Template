package frc.robot.subsystems.wrist;

import lib.MatchMode;
import lib.subsystem.BaseSubsystem;

public class WristSubsystem extends BaseSubsystem<WristIO, WristData> {
    public WristSubsystem(WristIO io) {
        super(io, WristData.class);
    }

    @Override
    public void onModeInit(MatchMode mode) {
        stop();
    }

    @Override
    public void periodicTelemetry() {
    }

    @Override
    public void stop() {
        io.halt();
    }
}