package frc.robot.subsystems.roller;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Subsystem;
import lib.MatchMode;
import lib.Utils;

public class RollerSubsystem extends Subsystem {
    private RollerIO io;

    public RollerSubsystem(RollerIO io) {
        this.io = io;
    }

    @Override
    public void onModeInit(MatchMode mode) { stop(); }

    @Override
    public void periodicTelemetry() {
        logger.log("Current Velocity RPM", io.getVelocityRPM());
        logger.log("Desired Velocity RPM", io.getDesiredRPM());
        logger.log("At Desired RPM", Utils.isWithin(io.getVelocityRPM(), io.getDesiredRPM(), 10d));
    }

    @Override
    public void stop() { io.halt(); }
    
    public Command setVelocityRPM(DoubleSupplier desiredRPM) {
        return runOnce(() -> io.setVelocity(desiredRPM.getAsDouble()))
                .withName("Set Roller Velocity to " + desiredRPM.getAsDouble() + " RPM");
    }

    public Command setPercentOutput(DoubleSupplier desiredPercent) {
        return runOnce(() -> io.setPercentOutput(desiredPercent.getAsDouble()))
                .withName("Set Roller Percent Output to " + desiredPercent.getAsDouble() * 100 + "%");
    }
}