package frc.robot.subsystems.roller.algae;

import edu.wpi.first.wpilibj2.command.Command;
import lib.MatchMode;
import lib.hardware.BeamSensor;
import lib.subsystem.BaseSubsystem;

public class AlgaeRollerSubsystem extends BaseSubsystem<AlgaeRollerIO, AlgaeRollerData> {
    private BeamSensor frontSensor;
    private BeamSensor backSensor;

    public AlgaeRollerSubsystem(AlgaeRollerIO io) {
        super(io, AlgaeRollerData.class);

        frontSensor = new BeamSensor(0);
        backSensor = new BeamSensor(1);
    }

    @Override
    public void onModeInit(MatchMode mode) {
        stop();
    }

    @Override
    public void periodicTelemetry() {
        this.data.map(AlgaeRollerData.DESIRED_ROLLER_RPM, io.getDesiredRPM());
        this.data.map(AlgaeRollerData.CURRENT_ROLLER_RPM, io.getVelocityRPM());
        this.data.map(AlgaeRollerData.HAS_ALGAE, frontSensor.isTripped() && !backSensor.isTripped());
    }

    @Override
    public void stop() {
        this.io.setVelocity(0d);
    }
    
    // =======================
    // =====  Commands  ======
    // =======================

    public Command setRollerVelocity(double velocity) {
        return run(() -> io.setVelocity(velocity));
    }

    public Command setRollerPercentOutput(double percent) {
        return run(() -> io.setPercentOutput(percent));
    }
}