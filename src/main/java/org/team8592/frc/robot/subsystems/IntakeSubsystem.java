package org.team8592.frc.robot.subsystems;

import org.team8592.lib.MatchMode;
import org.team8592.lib.hardware.motor.talonfx.KrakenX60Motor;

public class IntakeSubsystem extends NewtonSubsystem {
    private KrakenX60Motor outerRollerMotor, innerRollerMotor;

    protected IntakeSubsystem(boolean logToShuffleboard) {
        super(logToShuffleboard);

        this.outerRollerMotor = new KrakenX60Motor(0);
        this.innerRollerMotor = new KrakenX60Motor(0);
    }

    @Override
    public void onInit(MatchMode mode) {}

    @Override
    public void periodicLogs() {}

    @Override
    public void stop() {}
}