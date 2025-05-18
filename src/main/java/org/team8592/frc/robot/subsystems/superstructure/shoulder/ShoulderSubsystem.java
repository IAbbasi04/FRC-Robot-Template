package org.team8592.frc.robot.subsystems.superstructure.shoulder;

import java.util.function.DoubleSupplier;

import org.team8592.frc.robot.Robot;
import org.team8592.frc.robot.Constants.SHOULDER;
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
        return Utils.isWithin(this.getDegrees(), degrees, SHOULDER.SHOULDER_POSITION_TOLERANCE_DEGREES);
    }

    @Override
    public void onModeInit(MatchMode mode) {
        this.stop();
    }

    @Override
    public void periodicTelemetry() {
        logger.log("Current Degrees", this.io.getDegrees());
        logger.log("Target Degrees", this.targetDegrees);
        logger.log("At Target", this.atPosition());
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
        }).until(() -> atPosition())
        .finallyDo(() -> { // Simulation forgets to actually end the command
            if (Robot.isSimulation()) this.io.setPercentOutput(0d);
        });
    }

    public Command setPercentOutput(DoubleSupplier percent) {
        return this.run(() -> {
            this.io.setPercentOutput(percent.getAsDouble());
        });
    }
}