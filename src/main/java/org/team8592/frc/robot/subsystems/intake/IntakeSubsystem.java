package org.team8592.frc.robot.subsystems.intake;

import org.team8592.frc.robot.subsystems.NewtonSubsystem;
import org.team8592.lib.MatchMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static org.team8592.frc.robot.Constants.INTAKE.*;

public class IntakeSubsystem extends NewtonSubsystem {
    private IntakeIO io;

    private double desiredVelocityRPM = 0d;
    
    public IntakeCommands commands;

    public IntakeSubsystem(IntakeIO io, boolean logToShuffleboard) {
        super(logToShuffleboard);

        this.io = io;
        this.commands = new IntakeCommands(this);

        SmartDashboard.putData("Intake Gains", ROLLER_GAINS);
    }

    public void setPower(double percent) {
        this.desiredVelocityRPM = percent * 6000d;
        this.io.setSpeedPercentOutput(percent);
    }

    public void setVelocity(double velocityRPM) {
        this.desiredVelocityRPM = velocityRPM;
        this.io.setVelocityRPM(velocityRPM);
    }

    @Override
    public void onModeInit(MatchMode mode) {
        this.stop();
    }

    @Override
    public void periodicTelemetry() {
        logger.log("Intake Applied Voltage", this.io.getAppliedVoltage());
        logger.log("Intake Current Velocity RPM", this.io.getVelocityRPM());
        logger.log("Intake Desired Velocity RPM", this.desiredVelocityRPM);

        this.io.updateInputs();
    }

    @Override
    public void stop() {
        this.setVelocity(0);
    }
}