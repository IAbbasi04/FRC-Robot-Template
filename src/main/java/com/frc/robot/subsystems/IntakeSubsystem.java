package com.frc.robot.subsystems;

import com.frc.robot.Robot;
import com.lib.team8592.*;
import com.lib.team8592.hardware.motor.spark.SparkFlexMotor;
import com.lib.team8592.simulation.SimUtils;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeSubsystem extends NewtonSubsystem {
    private SparkFlexMotor topRollerMotor, bottomRollerMotor;
    private double desiredTopRPM = 0d;
    private double desiredBottomRPM = 0d;

    private DCMotorSim topIntakeSim = null;
    private DCMotorSim bottomIntakeSim = null;

    private Timer timer = new Timer();

    private TrapezoidProfile intakeProfile = new TrapezoidProfile(new Constraints(5000d, 5000d));

    private PIDProfile ROLLER_GAINS = new PIDProfile()
        .setP(1d)
        .setV(0d)
        .setMaxVelocity(5000d)
        .setMaxAcceleration(5000d);

    protected IntakeSubsystem(boolean logToShuffleboard) {
        super(logToShuffleboard);

        this.topRollerMotor = new SparkFlexMotor(29);
        this.bottomRollerMotor = new SparkFlexMotor(31);

        this.topRollerMotor.withGains(ROLLER_GAINS);
        this.bottomRollerMotor.withGains(ROLLER_GAINS);

        this.topIntakeSim = SimUtils.createSimSparkFlex(
            0.5d, 
            Utils.getMOIForRoller(2d, 1.5d)
        );

        this.bottomIntakeSim = SimUtils.createSimSparkFlex(
            0.5d, 
            Utils.getMOIForRoller(2d, 1d)
        );
    }

    public void setRollerVelocity(double topRPM, double bottomRPM) {
        if (!isEnabled()) return; // Do nothing if subsystem not active
        timer.start();

        this.desiredTopRPM = topRPM;
        this.desiredBottomRPM = bottomRPM;

        this.topRollerMotor.setVelocity(topRPM);
        this.bottomRollerMotor.setVelocity(bottomRPM);
    }

    private double getTopVoltageFromRPM() {
        return desiredTopRPM / 
            topRollerMotor.getMaxFreeVelocity() * 12d;
    }

    private double getBottomVoltageFromRPM() {
        return desiredTopRPM / 
            topRollerMotor.getMaxFreeVelocity() * 12d;
    }

    private double getCurrentTopVelocity() {
        if (Robot.isSimulation()) {
            return topIntakeSim.getAngularVelocityRPM() * 2d * topRollerMotor.getVoltageToRPMRatio();
        }
        return topRollerMotor.getVelocityRPM();
    }

    private double getCurrentBottomVelocity() {
        if (Robot.isSimulation()) {
            return bottomIntakeSim.getAngularVelocityRPM() * 2d * topRollerMotor.getVoltageToRPMRatio();
        }
        return bottomRollerMotor.getVelocityRPM();
    }

    @Override
    public void onInit(MatchMode mode) {
        this.stop();
        timer.reset();
        timer.stop();
    }

    @Override
    public void simulationPeriodic() {
        this.topIntakeSim.setInputVoltage(
            ROLLER_GAINS.toProfiledPIDController()
                .calculate(intakeProfile.calculate(
                    timer.get(),
                    new State(0, 0),
                    new State(0, 3000)
                ).velocity, getCurrentTopVelocity()
            ) / topRollerMotor.getVoltageToRPMRatio()
        );

        this.bottomIntakeSim.setInputVoltage(getBottomVoltageFromRPM());

        this.topIntakeSim.update(Robot.CLOCK.dt());
        this.bottomIntakeSim.update(Robot.CLOCK.dt());
    }

    @Override
    public void periodicLogs() {
        double desiredVelocity = intakeProfile.calculate(
            timer.get(),
            new State(0, 0),
            new State(0, 3000)
        ).velocity;

        this.logger.log("Desired Top Roller RPM", this.desiredTopRPM);
        this.logger.log("Desired Bottom Roller RPM", this.desiredBottomRPM);
        this.logger.log("Current Top Roller RPM", this.getCurrentTopVelocity());
        this.logger.log("Current Bottom Roller RPM", this.getCurrentBottomVelocity());
        this.logger.log("Top Roller Applied Voltage", this.getTopVoltageFromRPM());
        this.logger.log("Bottom Roller Applied Voltage", this.getBottomVoltageFromRPM());
        
        // TEMPORARY TESTS

        this.logger.log("TEST_Profile to 3000 RPM", desiredVelocity);
        this.logger.log("TEST_PID Response to timing", 
            ROLLER_GAINS.toProfiledPIDController()
                .calculate(desiredVelocity, getCurrentTopVelocity())
        );
    }

    @Override
    public void stop() {
        this.setRollerVelocity(0d, 0d);
    }
}