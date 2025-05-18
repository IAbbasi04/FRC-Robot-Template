package org.team8592.frc.robot.subsystems.superstructure.wrist;

import java.util.function.DoubleSupplier;

import org.team8592.frc.robot.Constants;
import org.team8592.frc.robot.Robot;
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
        return Utils.isWithin(getDegrees(), targetDegrees, Constants.WRIST.WRIST_POSITION_TOLERANCE_DEGREES);
    }

    @Override
    public void onModeInit(MatchMode mode) {
        stop();
    }

    @Override
    public void periodicTelemetry() {
        logger.log("Target Wrist Degrees", this.targetDegrees);
        logger.log("Current Wrist Degrees", io.getDegrees());
        logger.log("At Target Position", this.atPosition());

        io.updateInputs();
    }

    @Override
    public void stop() {
        this.io.setPercentOutput(0d);
    }

    // ========= Commands ======== \\

    public Command setDegrees(DoubleSupplier degrees) {
        return this.run(() -> {
            this.setDegrees(degrees.getAsDouble());
        }).until(() -> atPosition())
        .finallyDo(() -> { // Simulation forgets to actually end the command
            if (Robot.isSimulation()) this.io.setPercentOutput(0d);
        });
    }

    public Command setPercentPower(DoubleSupplier desiredPercent) {
        return run(() -> this.setPercentPower(desiredPercent.getAsDouble()));
    }
}