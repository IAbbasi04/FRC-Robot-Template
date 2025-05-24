package frc.robot.subsystems.climber;

import frc.robot.subsystems.Subsystem;
import lib.team1731.MatchMode;

public class ClimberSubsystem extends Subsystem {
    private ClimberIO io;

    public ClimberSubsystem(ClimberIO io) {
        this.io = io;
    }

    @Override
    public void onModeInit(MatchMode mode) {
        
    }

    @Override
    public void periodicTelemetry() {

    }

    @Override
    public void stop() {
        io.halt();
    }
}