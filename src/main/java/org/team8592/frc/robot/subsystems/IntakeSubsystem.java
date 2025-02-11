package org.team8592.frc.robot.subsystems;

import org.team8592.frc.robot.Constants.*;
import org.team8592.lib.MatchMode;
import org.team8592.lib.hardware.motor.talonfx.Falcon500Motor;

public class IntakeSubsystem extends NewtonSubsystem {
    // private KrakenX60Motor outerRollerMotor, innerRollerMotor;
    private Falcon500Motor falconMotor;

    private double desiredOuterVelocityRPM = 0d;
    private double desiredInnerVelocityRPM = 0d;

    protected IntakeSubsystem(boolean logToShuffleboard) {
        super(logToShuffleboard);

        falconMotor = new Falcon500Motor(0);

        // this.outerRollerMotor = new KrakenX60Motor(Ports.INTAKE_OUTER_ROLLER_CAN_ID);
        // this.innerRollerMotor = new KrakenX60Motor(Ports.INTAKE_INNER_ROLLER_CAN_ID);

        // this.outerRollerMotor.withGains(INTAKE.OUTER_ROLLER_GAINS);
        // this.innerRollerMotor.withGains(INTAKE.INNER_ROLLER_GAINS);
    }

    public void setRollerVelocity(double desiredOuterRPM) {
        if (!isEnabled()) return;

        this.desiredOuterVelocityRPM = desiredOuterRPM;

        this.falconMotor.setVelocity(desiredOuterRPM);
        // this.outerRollerMotor.setVelocity(desiredOuterRPM);
        // this.innerRollerMotor.setVelocity(desiredInnerRPM);
    }

    public double getOuterRollerVelocity() {
        // return this.outerRollerMotor.getVelocityRPM();
        return 0d;
    }

    public double getInnerRollerVelocity() {
        // return this.innerRollerMotor.getVelocityRPM();
        return 0d;
    }

    public boolean atTargetVelocity() {
        return (Math.abs(getInnerRollerVelocity() - desiredInnerVelocityRPM) <= INTAKE.AT_VELOCITY_THRESHOLD &&
            Math.abs(getOuterRollerVelocity() - desiredOuterVelocityRPM) <= INTAKE.AT_VELOCITY_THRESHOLD
        );
    }

    @Override
    public void onInit(MatchMode mode) {
        this.stop();
    }

    @Override
    public void periodicLogs() {
        this.logger.log("Current Inner Velocity RPM", this.getInnerRollerVelocity());
        this.logger.log("Current Outer Velocity RPM", this.getOuterRollerVelocity());
        this.logger.log("Desired Inner Roller RPM", this.desiredInnerVelocityRPM);
        this.logger.log("Desired Outer Roller RPM", this.desiredOuterVelocityRPM);
    }

    @Override
    public void stop() {
        this.setRollerVelocity(0d);
    }
}