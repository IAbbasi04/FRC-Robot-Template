package frc.robot.subsystems.rollers;

import lib.MatchMode;
import lib.hardware.motor.MotorConstants;
import lib.hardware.motor.rev.SparkMaxMotor;
import lib.subsystem.BaseSubsystem;

import static frc.robot.subsystems.rollers.RollerConstants.*;

import edu.wpi.first.wpilibj2.command.Command;

public class RollerSubsystem extends BaseSubsystem {
    private SparkMaxMotor rollerMotor;
    private double desiredVelocityRPM = 0.0;

    public RollerSubsystem() {
        this.rollerMotor = new SparkMaxMotor(MOTOR_CONFIG);
        this.rollerMotor.withGains(VELOCITY_GAINS);
    }

    @Override
    public void onModeInit(MatchMode mode) {
        this.stop();
    }

    @Override
    public void periodicTelemetry() {
        this.logger.log("Desired Velocity RPM", desiredVelocityRPM);
        this.logger.log("Current Velocity RPM", rollerMotor.getVelocityRPM());
    }

    @Override
    public void stop() {
        rollerMotor.setPercentOutput(0);
    }
    
    public Command setRollerVelocity(double desiredRPM) {
        return run(() -> {
            this.desiredVelocityRPM = desiredRPM;
            rollerMotor.setVelocity(desiredRPM);
        });
    }

    public Command setRollerPercent(double desiredPercent) {
        return run(() -> {
            this.desiredVelocityRPM = desiredPercent * MotorConstants.NEO.MAX_VELOCITY_RPM;
            rollerMotor.setPercentOutput(desiredPercent);
        });
    }

}
