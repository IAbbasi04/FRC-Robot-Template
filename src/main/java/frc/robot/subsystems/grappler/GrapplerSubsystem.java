package frc.robot.subsystems.grappler;

import edu.wpi.first.wpilibj2.command.Command;
import lib.MatchMode;
import lib.subsystem.BaseSubsystem;

public class GrapplerSubsystem extends BaseSubsystem<GrapplerIO, GrapplerData> {
    public GrapplerSubsystem(GrapplerIO io) {
        super(io, GrapplerData.class);
    }

    @Override
    public void onModeInit(MatchMode mode) {
        stop();
    }

    @Override
    public void periodicTelemetry() {
        
    }

    @Override
    public void stop() {
        io.halt();
    }
    
    public Command grabCage() {
        return run(() -> io.setVelocity(3000));
    }

    public Command spitCage() {
        return run(() -> io.setVelocity(-3000));
    }
}
