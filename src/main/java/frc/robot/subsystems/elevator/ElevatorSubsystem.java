package frc.robot.subsystems.elevator;

import lib.MatchMode;
import lib.hardware.motor.rev.SparkMaxMotor;
import lib.subsystem.BaseSubsystem;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorSubsystem extends BaseSubsystem {
    private SparkMaxMotor elevatorMotor;
    private double desiredInches = 0.0;

    public ElevatorSubsystem() {
        this.elevatorMotor = new SparkMaxMotor(MOTOR_CONFIG);
        this.elevatorMotor.withGains(RAISE_GAINS, LOWER_GAINS);
    }

    public double getHeightInches() {
        return elevatorMotor.getRotations() * PULLEY_CIRCUMFERENCE_INCHES * GEAR_RATIO;
    }

    @Override
    public void onModeInit(MatchMode mode) {
        this.stop();
    }

    @Override
    public void periodicTelemetry() {
        this.logger.log("Desired Height Inches", desiredInches);
        this.logger.log("Current Height Inches", getHeightInches());
    }

    @Override
    public void stop() {
        this.desiredInches = getHeightInches();
        elevatorMotor.setPosition((getHeightInches() / PULLEY_CIRCUMFERENCE_INCHES) / GEAR_RATIO);
    }
    
    public Command setHeightInches(double desiredInches) {
        return run(() -> {
            this.desiredInches = desiredInches;
            int desiredPIDSlot = desiredInches > getHeightInches() ? RAISE_GAINS.pidSlot : LOWER_GAINS.pidSlot;
            elevatorMotor.setPosition((desiredInches / PULLEY_CIRCUMFERENCE_INCHES) / GEAR_RATIO, desiredPIDSlot);
        });
    }
}