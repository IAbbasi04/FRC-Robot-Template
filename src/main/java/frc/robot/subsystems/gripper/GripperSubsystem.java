package frc.robot.subsystems.gripper;

import frc.robot.subsystems.Subsystem;
import lib.team1731.MatchMode;

public class GripperSubsystem extends Subsystem {
    private GripperIO io;

    public GripperSubsystem(GripperIO io) {
        this.io = io;
    }

    @Override
    public void onModeInit(MatchMode mode) {
        stop();
    }

    @Override
    public void periodicTelemetry() {
        logger.log("Current Angle Degrees", io.getDegrees());
        logger.log("Desired Angle Degrees", io.getDesiredDegrees());
        logger.log("At Angle Degrees", io.atAngleDegrees());
    }

    @Override
    public void stop() {
        io.halt();
    }
}