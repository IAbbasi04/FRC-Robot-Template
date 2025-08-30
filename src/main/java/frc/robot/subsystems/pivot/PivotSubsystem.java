package frc.robot.subsystems.pivot;

import lib.MatchMode;
import lib.hardware.motor.rev.SparkMaxMotor;
import lib.subsystem.BaseSubsystem;

import static frc.robot.subsystems.pivot.PivotConstants.*;

import edu.wpi.first.wpilibj2.command.Command;

public class PivotSubsystem extends BaseSubsystem {
    private SparkMaxMotor pivotMotor;
    private double desiredRotations = 0.0;

    public PivotSubsystem() {
        this.pivotMotor = new SparkMaxMotor(MOTOR_CONFIG);
        this.pivotMotor.withGains(LOWER_GAINS, RAISE_GAINS);
    }

    public double getRotations() {
        return pivotMotor.getRotations();
    }

    @Override
    public void onModeInit(MatchMode mode) {
    }

    @Override
    public void periodicTelemetry() {
        this.logger.log("Current Rotations", getRotations());
        this.logger.log("Desired Rotations", this.desiredRotations);
    }

    @Override
    public void stop() {
        this.desiredRotations = getRotations();
        this.pivotMotor.setPosition(getRotations());
    }

    public Command setRotations(double rotations) {
        return run(() -> {
            this.desiredRotations = rotations;
            int desiredPIDSlot = rotations > getRotations() ? RAISE_GAINS.pidSlot : LOWER_GAINS.pidSlot;
            pivotMotor.setPosition(rotations, desiredPIDSlot);
        });
    }
    
}