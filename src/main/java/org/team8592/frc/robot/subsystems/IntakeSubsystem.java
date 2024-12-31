package org.team8592.frc.robot.subsystems;

import org.team8592.frc.robot.Ports;
import org.team8592.lib.MatchMode;
import org.team8592.lib.PIDProfile;
import org.team8592.lib.hardware.motor.NewtonMotor;
import org.team8592.lib.hardware.motor.spark.SparkFlexMotor;

public class IntakeSubsystem extends NewtonSubsystem {
    private NewtonMotor topRollerMotor, bottomRollerMotor;

    private double desiredTopVelocityRPM = 0d;
    private double desiredBottomVelocityRPM = 0d;

    private PIDProfile topRollerGains = new PIDProfile()
        .setP(1E-3)
        .setFeedForward(1.5, 0.01, 0.1)
        .setMaxVelocity(5000d)
        .setMaxAcceleration(5000d)
        ;

    private PIDProfile bottomRollerGains = new PIDProfile()
        .setP(1E-3)
        .setFeedForward(1.5, 0.01, 0.1)
        .setMaxVelocity(5000d)
        .setMaxAcceleration(5000d)
        ;

    protected IntakeSubsystem(boolean logToShuffleboard) {
        super(logToShuffleboard);

        this.topRollerMotor = new SparkFlexMotor(Ports.INTAKE_TOP_ROLLER_CAN_ID);
        this.bottomRollerMotor = new SparkFlexMotor(Ports.INTAKE_BOTTOM_ROLLER_CAN_ID);

        this.topRollerMotor.withGains(topRollerGains);
        this.bottomRollerMotor.withGains(bottomRollerGains);
    }

    public void setRollerVelocity(double desiredTopRPM, double desiredBottomRPM) {
        if (!isEnabled()) return; // Only run if subsystem is in active subsystem list

        this.desiredTopVelocityRPM = desiredTopRPM;
        this.desiredBottomVelocityRPM = desiredBottomRPM;

        this.topRollerMotor.setVelocity(desiredTopRPM);
        this.bottomRollerMotor.setVelocity(desiredBottomRPM);
    }

    public double getTopRollerVelocity() {
        return this.topRollerMotor.getVelocityRPM();
    }

    public double getBottomRollerVelocity() {
        return this.bottomRollerMotor.getVelocityRPM();
    }

    @Override
    public void onInit(MatchMode mode) {}

    @Override
    public void periodicLogs() {
        this.logger.log("Desired Top Roller RPM", this.desiredTopVelocityRPM);
        this.logger.log("Desired Bottom Roller RPM", this.desiredBottomVelocityRPM);
        this.logger.log("Current Top Roller RPM", this.getTopRollerVelocity());
        this.logger.log("Current Bottom Roller RPM", this.getBottomRollerVelocity());
    }

    @Override
    public void stop() {
        this.setRollerVelocity(0d, 0d);
    }
}