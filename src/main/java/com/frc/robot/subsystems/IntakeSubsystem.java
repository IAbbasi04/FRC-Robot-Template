package com.frc.robot.subsystems;

import com.lib.team8592.MatchMode;
import com.lib.team8592.PIDProfile;
import com.lib.team8592.hardware.NewtonFeedForward;
import com.lib.team8592.hardware.motor.SparkFlexMotorController;

public class IntakeSubsystem extends NewtonSubsystem {
    private SparkFlexMotorController topRollerMotor, bottomRollerMotor;
    private double desiredTopRPM = 0d;
    private double desiredBottomRPM = 0d;

    private PIDProfile ROLLER_GAINS = new PIDProfile()
        .setP(1E-3)
        .setFeedForward(new NewtonFeedForward().setV(1.5));

    protected IntakeSubsystem(boolean logToShuffleboard) {
        super(logToShuffleboard);

        this.topRollerMotor = new SparkFlexMotorController(29);
        this.bottomRollerMotor = new SparkFlexMotorController(31);
    }

    public void setRollerVelocity(double topRPM, double bottomRPM) {
        if (!isEnabled()) return; // Do nothing if subsystem not active

        this.desiredTopRPM = topRPM;
        this.desiredBottomRPM = bottomRPM;

        this.topRollerMotor.setVelocity(topRPM);
        this.bottomRollerMotor.setVelocity(bottomRPM);
    }

    @Override
    public void onInit(MatchMode mode) {}

    @Override
    public void periodicLogs() {
        this.logger.log("Desired Top Roller RPM", this.desiredTopRPM);
        this.logger.log("Desired Bottom Roller RPM", this.desiredBottomRPM);
        this.logger.log("Current Top Roller RPM", this.topRollerMotor.getVelocityRPM());
        this.logger.log("Current Bottom Roller RPM", this.bottomRollerMotor.getVelocityRPM());
    }

    @Override
    public void stop() {
        this.setRollerVelocity(0d, 0d);
    }
}