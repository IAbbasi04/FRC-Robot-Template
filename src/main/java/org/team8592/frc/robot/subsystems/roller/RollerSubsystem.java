package org.team8592.frc.robot.subsystems.roller;

import org.team8592.frc.robot.subsystems.NewtonSubsystem;
import org.team8592.lib.MatchMode;

import edu.wpi.first.wpilibj2.command.Command;


public class RollerSubsystem extends NewtonSubsystem {
    private RollerIO io;
    private double desiredVelocityRPM = 0d;

    public RollerSubsystem(RollerIO io) {
        this.io = io;
    }

    private void setRollerVelocity(double desiredRPM) {
        this.desiredVelocityRPM = desiredRPM;
        this.io.setVelocityRPM(desiredRPM);
    }

    private void setRollerPercent(double desiredPercent) {
        this.io.setPercentOutput(desiredPercent);
        this.desiredVelocityRPM = desiredPercent * io.getMaxVelocityRPM();
    }

    @Override
    public void onModeInit(MatchMode mode) {
        this.stop();
    }

    @Override
    public void periodicTelemetry() {
        logger.log("Desired Roller RPM", desiredVelocityRPM);
        logger.log("Current Roller RPM", io.getVelocityRPM());
        logger.log("Current Roller Volts", io.getVoltage());
        this.io.updateInputs();
    }

    @Override
    public void stop() {
        this.setRollerPercent(0d);
    }

    // ============ Commands =========== \\

    public Command setVelocity(double desiredRPM) {
        return run(() -> {
            this.setRollerVelocity(desiredRPM);
        });
    }

    public Command setPercentPower(double desiredPercent) {
        return run(() -> {
            this.setRollerPercent(desiredPercent);
        });
    }
}