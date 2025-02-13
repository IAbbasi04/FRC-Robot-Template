package org.team8592.frc.robot.subsystems.intake;

import org.team8592.frc.robot.subsystems.NewtonSubsystem;
import org.team8592.frc.robot.subsystems.SubsystemCommands;
import org.team8592.lib.MatchMode;

public class IntakeSubsystem extends NewtonSubsystem<SubsystemCommands<?>> {
    private IntakeIO intakeIO;

    public IntakeSubsystem(IntakeIO intakeIO, boolean logToShuffleboard) {
        super(logToShuffleboard);

        this.intakeIO = intakeIO;
    }

    public void setPower(double percent) {
        intakeIO.setSpeedPercentOutput(percent);
    }

    @Override
    public void onModeInit(MatchMode mode) {
        this.stop();
    }

    @Override
    public void periodicTelemetry() {
        logger.log("Intake Applied Voltage", intakeIO.getAppliedVoltage());
        logger.log("Intake Velocity RPM", intakeIO.getVelocityRPM());
        logger.log("Intake Desired Percent", intakeIO.getDesiredPercent());
    }

    @Override
    public void stop() {
        this.setPower(0);
    }
}