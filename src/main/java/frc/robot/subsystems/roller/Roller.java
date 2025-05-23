package frc.robot.subsystems.roller;

import frc.robot.subsystems.Subsystem;
import lib.team1731.MatchMode;

public class Roller extends Subsystem {
    private RollerIO io;
    public Roller(RollerIO io) {
        this.io = io;
    }

    @Override
    public void onModeInit(MatchMode mode) {
        stop();
    }

    @Override
    public void periodicTelemetry() {
        logger.log("Current RPM", io.getVelocityRPM());
        logger.log("Desired RPM", io.getDesiredRPM());
        logger.log("Output Voltage", io.getOutputVoltage());
    }

    @Override
    public void stop() {
        io.halt();
    }
}