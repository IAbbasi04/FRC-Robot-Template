package org.team8592.frc.robot.subsystems.superstructure.wrist;

import org.team8592.frc.robot.subsystems.NewtonSubsystem;
import org.team8592.lib.MatchMode;

import edu.wpi.first.wpilibj2.command.Command;

public class WristSubsystem extends NewtonSubsystem {
    private WristIO io;

    public WristSubsystem(WristIO io) {
        this.io = io;
    }

    @Override
    public void onModeInit(MatchMode mode) {
        stop();
    }

    @Override
    public void periodicTelemetry() {
        logger.log("Current Wrist Degrees", io.getDegrees());
        logger.log("Target Wrist Degrees", io.getTargetDegrees());
        logger.log("At Target Position", io.atTargetPosition());

        logger.log("Target Voltage", io.getTargetVoltage());
        logger.log("Current Voltage", io.getVoltage());
        
        logger.log("Current Velocity RPM", io.getVelocityRPM());

        io.updateInputs();
    }

    @Override
    public void stop() {
        this.io.setPercentOutput(0d);
    }

    // ========= Commands ======== \\

    public Command setDegrees(double desiredDegrees) {
        return run(() -> io.setDegrees(desiredDegrees)).until(() -> io.atTargetPosition());
    }

    public Command setPercentPower(double desiredPercent) {
        return run(() -> io.setPercentOutput(desiredPercent));
    }
}