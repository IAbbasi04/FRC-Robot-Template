package com.lib.team8592.hardware;

import com.lib.team8592.PIDProfile;
import com.lib.team8592.hardware.motor.NewtonMotor;
import com.lib.team8592.logging.SmartLogger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.*;

import edu.wpi.first.util.sendable.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.*;

public class SimIO implements Sendable {
    private DCMotorSim simMotor;
    private EncoderSim simEncoder;

    private PIDProfile pidGains;
    private ProfiledPIDController pidCtrl;
    private NewtonMotor motor;
    
    private double lastDesiredOutput = 0d;
    private double goalOutput = 0d;
    private double appliedVoltage = 0d;

    private State currentState = new State();
    private State goalState = new State();

    public enum ControlMode {
        kVelocity,
        kPosition,
        kDutyCycle,
        kVoltage,
        kOff
    }

    private ControlMode controlMode = ControlMode.kOff;
    private ControlMode lastControlMode = ControlMode.kOff;

    private TrapezoidProfile motionProfile;

    private Timer motionTimer = new Timer();

    private static int instances = 0;

    private SmartLogger logger;

    public SimIO(double gearRatio, double momentOfInertia, NewtonMotor motor) {
        this.motor = motor;
        this.simMotor = new DCMotorSim(
            NewtonMotor.getDCMotor(motor, 1), 
            gearRatio, 
            momentOfInertia
        );

        this.pidGains = motor.getPIDGains().get(0);
        this.pidCtrl = pidGains.toProfiledPIDController();

        this.simEncoder = EncoderSim.createForIndex(motor.getDeviceID());
        this.simEncoder.setPeriod(0.02);
        this.simEncoder.setDistancePerPulse(motor.getVoltageToRPMRatio());

        this.motionProfile = new TrapezoidProfile(
            new Constraints(
                pidGains.maxVelocity,
                pidGains.maxAcceleration
            )
        );

        instances++;

        this.logger = new SmartLogger("SimIO[" + instances + "]");
    }

    public void setVelocityRPM(double desiredRPM) {
        this.controlMode = ControlMode.kVelocity;
        this.goalOutput = desiredRPM;
        
        double desiredVoltage = this.goalOutput / motor.getVoltageToRPMRatio();

        currentState = new State(
            0, 
            simMotor.getAngularVelocityRPM() / 
            motor.getVoltageToRPMRatio()
        );

        goalState = new State(
            0,
            desiredVoltage
        );

        double appliedVelocity = this.pidCtrl.calculate(currentState.velocity, goalState.velocity);
        this.appliedVoltage = appliedVelocity / motor.getVoltageToRPMRatio();

        this.simMotor.setInputVoltage(appliedVelocity);
    }

    public void setRotations(double desiredRotations) {
        this.controlMode = ControlMode.kPosition;
        this.goalOutput = desiredRotations;

        currentState = new State(
            simMotor.getAngularPositionRotations(), 
            simMotor.getAngularVelocityRPM()
        );

        goalState = new State(goalOutput, 0d);

        appliedVoltage = motionProfile.calculate(
            motionTimer.get(), 
            currentState, 
            goalState
        ).velocity / motor.getVoltageToRPMRatio();

        this.motor.setVoltage(appliedVoltage);
        this.simMotor.setInputVoltage(appliedVoltage);
    }

    public void setVoltage(double desiredVolts) {
        this.controlMode = ControlMode.kVoltage;
        this.goalOutput = desiredVolts;

        double desiredVoltage = this.goalOutput / motor.getVoltageToRPMRatio();

        currentState = new State(
            0, 
            simMotor.getAngularVelocityRPM() / 
            motor.getVoltageToRPMRatio()   
        );

        goalState = new State(
            0,
            desiredVoltage
        );

        appliedVoltage = motionProfile.calculate(
            motionTimer.get(), 
            currentState, 
            goalState
        ).velocity;

        this.motor.setVoltage(appliedVoltage);
        this.simMotor.setInputVoltage(appliedVoltage);
    }

    public void setDutyCycle(double desiredPercent) {
        this.controlMode = ControlMode.kDutyCycle;
        this.goalOutput = desiredPercent;

        double goalVoltage = this.goalOutput / motor.getVoltageToRPMRatio();

        currentState = new State(
            0, 
            simMotor.getAngularVelocityRPM() / 
            motor.getVoltageToRPMRatio()   
        );

        goalState = new State(
            0,
            goalVoltage
        );
        
        this.appliedVoltage = pidCtrl.calculate(currentState.velocity, goalState.velocity);

        this.motor.setVoltage(appliedVoltage);
        this.simMotor.setInputVoltage(appliedVoltage);
    }

    public void update(double dt) {
        if (lastDesiredOutput != goalOutput ||
            !lastControlMode.equals(controlMode)) {
            motionTimer.restart();
        }

        this.logger.log("Current Voltage", this.currentState.velocity);
        this.logger.log("Goal Voltage", this.goalState.velocity);
        this.logger.log("Applied Voltage", this.appliedVoltage);
        this.logger.log("Control Mode", this.controlMode);
        this.logger.log("Goal Output No-Units", this.goalOutput);
        this.logger.log("Current Motor Position", this.simMotor.getAngularPositionRotations());
        this.logger.log("Current Motor Velocity", this.simMotor.getAngularVelocityRPM());
        this.logger.log("Motion Profile Time", motionTimer.get());
        
        this.simEncoder.setDistance(simMotor.getAngularPositionRotations());
        this.simMotor.update(dt);

        this.lastDesiredOutput = goalOutput;
        this.lastControlMode = controlMode;
    }

    @Override
    public void initSendable(SendableBuilder builder) {}
}