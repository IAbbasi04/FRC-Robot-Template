package frc.robot.subsystems.wrist;

import lib.MatchMode;
import lib.hardware.motor.rev.SparkMaxMotor;
import lib.subsystem.BaseSubsystem;

import static frc.robot.subsystems.wrist.WristConstants.*;

import edu.wpi.first.wpilibj2.command.Command;

public class WristSubsystem extends BaseSubsystem {
    private SparkMaxMotor wristMotor;
    private double desiredDegrees = 0.0;

    public WristSubsystem() {
        this.wristMotor = new SparkMaxMotor(WRIST_MOTOR_CONFIG);
        this.wristMotor.withGains(WRIST_GAINS);
    }

    public double getDegrees() {
        return wristMotor.getRotations() * 360 * WRIST_GEAR_RATIO;
    }

    @Override
    public void onModeInit(MatchMode mode) {
        this.stop();
    }

    @Override
    public void periodicTelemetry() {
        this.logger.log("Desired Degrees", desiredDegrees);
        this.logger.log("Current Degrees", getDegrees());
    }

    @Override
    public void stop() {
        this.desiredDegrees = getDegrees();
        wristMotor.setPosition((getDegrees() / 360d) / WRIST_GEAR_RATIO);
    }
    
    public Command setWristDegrees(double desiredDegrees) {
        return run(() -> {
            this.desiredDegrees = desiredDegrees;
            wristMotor.setPosition((desiredDegrees / 360d) / WRIST_GEAR_RATIO);
        });
    }
}