package org.team8592.frc.robot.subsystems.superstructure.shoulder;

import java.util.function.DoubleSupplier;

import org.team8592.frc.robot.subsystems.NewtonSubsystem;
import org.team8592.lib.MatchMode;
import org.team8592.lib.Utils;

import edu.wpi.first.wpilibj2.command.Command;

public class ShoulderSubsystem extends NewtonSubsystem {
    private ShoulderIO io;
    private double targetDegrees = 0d;

    public ShoulderSubsystem(ShoulderIO io) {
        this.io = io;
    }

    private void hold() {
        this.io.setDegrees(io.getDegrees());
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
        this.stop();
    }

    @Override
    public void periodicTelemetry() {
        logger.log("Current Degrees", io.getDegrees());
        logger.log("Target Degrees", targetDegrees);
        this.io.updateInputs();
    }

    @Override
    public void stop() {
        this.hold();
    }

    // ================================
    //            Commands
    // ================================

    public Command setDegrees(DoubleSupplier degrees) {
        return this.run(() -> {
            this.setDegrees(degrees.getAsDouble());
        }).until(() -> atPosition());
    }

    public Command setPercentOutput(DoubleSupplier percent) {
        return this.run(() -> {
            this.io.setPercentOutput(percent.getAsDouble());
        });
    }
}