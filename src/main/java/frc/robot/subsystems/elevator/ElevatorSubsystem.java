package frc.robot.subsystems.elevator;

import lib.MatchMode;
import lib.subsystem.BaseSubsystem;

public class ElevatorSubsystem extends BaseSubsystem<ElevatorIO, ElevatorData> {
    public ElevatorSubsystem(ElevatorIO io) {
        super(io, ElevatorData.class);
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