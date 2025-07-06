package frc.robot.subsystems.roller.coral;

import edu.wpi.first.wpilibj2.command.Command;
import lib.MatchMode;
import lib.hardware.BeamSensor;
import lib.subsystem.BaseSubsystem;

public class CoralRollerSubsystem extends BaseSubsystem<CoralRollerIO, CoralRollerData> {
    private BeamSensor frontSensor;
    private BeamSensor backSensor;

    public CoralRollerSubsystem(CoralRollerIO io) {
        super(io, CoralRollerData.class);

        frontSensor = new BeamSensor(0);
        backSensor = new BeamSensor(1);
    }

    @Override
    public void onModeInit(MatchMode mode) {
        stop();
    }

    @Override
    public void periodicTelemetry() {
        this.data.map(CoralRollerData.DESIRED_ROLLER_RPM, io.getDesiredRPM());
        this.data.map(CoralRollerData.CURRENT_ROLLER_RPM, io.getVelocityRPM());
        this.data.map(CoralRollerData.HAS_CORAL, frontSensor.isTripped() && !backSensor.isTripped());
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