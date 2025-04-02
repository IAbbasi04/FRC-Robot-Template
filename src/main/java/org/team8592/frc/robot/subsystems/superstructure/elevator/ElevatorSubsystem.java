package org.team8592.frc.robot.subsystems.superstructure.elevator;

import java.util.function.DoubleSupplier;

import org.team8592.frc.robot.subsystems.NewtonSubsystem;
import org.team8592.lib.MatchMode;

import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorSubsystem extends NewtonSubsystem {
    private ElevatorIO io;
    private double desiredInches = 0d;
    
    public ElevatorSubsystem(ElevatorIO io) {
        this.io = io;
    }

    @Override
    public void onModeInit(MatchMode mode) {
        stop();
    }

    @Override
    public void periodicTelemetry() {
        logger.log("Current Inches", io.getInches());
        logger.log("Desired Inches", desiredInches);
    }

    @Override
    public void stop() {
        io.setInches(io.getInches());
    }

    public Command setInches(DoubleSupplier desiredInches) {
        return this.run(() -> {
            this.desiredInches = desiredInches.getAsDouble();
            io.setInches(this.desiredInches);
        });
    }

    public Command setPercent(DoubleSupplier percent) {
        return this.run(() -> {
            io.setPercentOutput(percent.getAsDouble());
        });
    }
}