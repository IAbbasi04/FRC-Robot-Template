package org.team8592.frc.robot.subsystems.superstructure.wrist;

import java.util.function.DoubleSupplier;

import org.team8592.frc.robot.subsystems.NewtonSubsystem;
import org.team8592.lib.MatchMode;
import org.team8592.lib.Utils;

import edu.wpi.first.wpilibj2.command.Command;

public class WristSubsystem extends NewtonSubsystem {
    private WristIO io;
    private double targetDegrees = 0.0;

    public WristSubsystem(WristIO io) {
        this.io = io;
    }

    private void setPercentPower(double percent) {
        this.io.setPercentOutput(percent);
    }

    private void setDegrees(double degrees) {
        this.targetDegrees = degrees;
        this.io.setDegrees(degrees);
    }

    public double getDegrees() {
        return this.io.getDegrees();
    }

    public boolean atPosition() {
        return atPosition(this.targetDegrees);
    }

    public boolean atPosition(double degrees) {
        return Utils.isWithin(this.getDegrees(), degrees, 0.25);
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

    public Command setDegrees(DoubleSupplier desiredDegrees) {
        return run(() -> this.setDegrees(desiredDegrees.getAsDouble())).until(() -> io.atTargetPosition());
    }

    public Command setPercentPower(DoubleSupplier desiredPercent) {
        return run(() -> this.setPercentPower(desiredPercent.getAsDouble()));
    }
}