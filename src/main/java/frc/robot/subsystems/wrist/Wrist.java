package frc.robot.subsystems.wrist;

import frc.robot.subsystems.Subsystem;
import lib.team1731.MatchMode;

public class Wrist extends Subsystem {
    private WristIO io;
    public Wrist(WristIO io) {
        this.io = io;
    }

    @Override
    public void onModeInit(MatchMode mode) {
        if (mode.equals(MatchMode.DISABLED)) stop();
    }

    @Override
    public void periodicTelemetry() {
        logger.log("Current Angle", io.getDegrees());
        logger.log("Desired Angle", io.getDesiredDegrees());
    }

    @Override
    public void stop() {
        io.halt();
    }
}