package frc.robot.subsystems.elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Subsystem;
import lib.team1731.MatchMode;

public class Elevator extends Subsystem {
    private ElevatorIO io;

    public Elevator(ElevatorIO io) {
        this.io = io;
    }

    @Override
    public void onModeInit(MatchMode mode) {
        stop();
    }

    @Override
    public void periodicTelemetry() {
        io.updateInputs();
        logger.log("Current Inches", io.getCurrentInches());
        logger.log("Desired Inches", io.getDesiredInches());
        logger.log("At Position", io.atPosition());
    }

    @Override
    public void stop() {
        io.halt();
    }

    // ========================== \\
    // ======== Commands ======== \\
    // ========================== \\

    public Command setInches(DoubleSupplier inches) {
        return this.run(() -> io.setInches(inches.getAsDouble()));//.until(() -> io.atPosition());
    }
}