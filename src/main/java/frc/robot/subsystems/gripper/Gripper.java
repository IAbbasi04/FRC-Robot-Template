package frc.robot.subsystems.gripper;

import frc.robot.subsystems.Subsystem;
import lib.team1731.MatchMode;

public class Gripper extends Subsystem {
    private GripperIO io;

    public Gripper(GripperIO io) {
        this.io = io;
    }

    @Override
    public void onModeInit(MatchMode mode) {
        io.halt();
    }

    @Override
    public void periodicTelemetry() {
        logger.log("Current Angle Degrees", io.getAngle());
        logger.log("Desired Angle Degrees", io.getDesiredAngle());
    }

    @Override
    public void stop() {

    }
}