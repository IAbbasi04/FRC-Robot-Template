package com.lib.team8592.hardware;

import com.lib.team8592.PIDProfile;
import com.lib.team8592.hardware.motor.NewtonMotor;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.*;

import edu.wpi.first.util.sendable.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.*;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class SimIO implements Sendable {
    private DCMotorSim simMotor;
    private EncoderSim simEncoder;

    private PIDProfile pidGains;
    private ProfiledPIDController pidCtrl;
    private NewtonMotor motor;
    
    private double lastDesiredOutput = 0d;
    private double desiredOutput = 0d;
    private double appliedVoltage = 0d;

    private State currentState = new State();
    private State desiredState = new State();

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
        this.simEncoder.setDistancePerPulse(1d);

        this.motionProfile = new TrapezoidProfile(
            new Constraints(
                pidGains.maxVelocity,
                pidGains.maxAcceleration
            )
        );
    }

    public void setVelocityRPM(double desiredRPM) {
        this.controlMode = ControlMode.kVelocity;
        this.desiredOutput = desiredRPM;
        
        double desiredVoltage = this.desiredOutput / motor.getVoltageToRPMRatio();

        currentState = new State(
            0, 
            simMotor.getAngularVelocityRPM() / 
            motor.getVoltageToRPMRatio()
        );

        desiredState = new State(
            0,
            desiredVoltage
        );

        // appliedVoltage = motionProfile.calculate(
        //     motionTimer.get(), 
        //     currentState, 
        //     desiredState
        // ).velocity;

        this.pidCtrl.setGoal(desiredState.velocity);

        SmartDashboard.putNumber("MMLLIIAASLDJKJDKLAS", this.pidCtrl.calculate(currentState.velocity));

        this.appliedVoltage = this.pidCtrl.calculate(currentState.velocity);

        this.motor.setVoltage(appliedVoltage);
        this.simMotor.setInputVoltage(appliedVoltage);
    }

    public void setRotations(double desiredRotations) {
        this.controlMode = ControlMode.kPosition;
        this.desiredOutput = desiredRotations;

        currentState = new State(
            simMotor.getAngularPositionRotations(), 
            simMotor.getAngularVelocityRPM()
        );

        desiredState = new State(desiredOutput, 0d);

        appliedVoltage = motionProfile.calculate(
            motionTimer.get(), 
            currentState, 
            desiredState
        ).velocity / motor.getVoltageToRPMRatio();

        this.motor.setVoltage(appliedVoltage);
        this.simMotor.setInputVoltage(appliedVoltage);
    }

    public void setVoltage(double desiredVolts) {
        this.controlMode = ControlMode.kVoltage;
        this.desiredOutput = desiredVolts;

        double desiredVoltage = this.desiredOutput / motor.getVoltageToRPMRatio();

        currentState = new State(
            0, 
            simMotor.getAngularVelocityRPM() / 
            motor.getVoltageToRPMRatio()   
        );

        desiredState = new State(
            0,
            desiredVoltage
        );

        appliedVoltage = motionProfile.calculate(
            motionTimer.get(), 
            currentState, 
            desiredState
        ).velocity;

        this.motor.setVoltage(appliedVoltage);
        this.simMotor.setInputVoltage(appliedVoltage);
    }

    public void setDutyCycle(double desiredPercent) {
        this.controlMode = ControlMode.kDutyCycle;
        this.desiredOutput = desiredPercent;

        double desiredVoltage = this.desiredOutput / motor.getVoltageToRPMRatio();

        currentState = new State(
            0, 
            simMotor.getAngularVelocityRPM() / 
            motor.getVoltageToRPMRatio()   
        );

        desiredState = new State(
            0,
            desiredVoltage
        );

        appliedVoltage = motionProfile.calculate(
            motionTimer.get(), 
            currentState, 
            desiredState
        ).velocity;

        this.motor.setVoltage(appliedVoltage);
        this.simMotor.setInputVoltage(appliedVoltage);
    }

    public void updateTimer() {
        if (lastDesiredOutput != desiredOutput ||
            !lastControlMode.equals(controlMode)) {
            motionTimer.restart();
        }
    }

    public void update(double dt) {
        this.updateTimer();

        SmartDashboard.putNumber("SIMIO/CURRENT VOLTAGE", currentState.velocity);
        SmartDashboard.putNumber("SIMIO/DESIRED VOLTAGE", desiredState.velocity);
        SmartDashboard.putNumber("SIMIO/APPLIED VOLTAGE", appliedVoltage);
        SmartDashboard.putNumber("SIMIO/DESIRED OUTPUT", desiredOutput);
        SmartDashboard.putString("SIMIO/CONTROL MODE", controlMode.name());
        SmartDashboard.putNumber("SIMIO/Position", simMotor.getAngularPositionRotations());
        SmartDashboard.putNumber("SIMIO/Velocity", simMotor.getAngularVelocityRPM());
        SmartDashboard.putNumber("SIMIO/Encoder", simEncoder.getDistance());
        
        this.simEncoder.setDistance(simMotor.getAngularPositionRotations());
        this.simMotor.update(dt);

        this.lastDesiredOutput = desiredOutput;
        this.lastControlMode = controlMode;
    }

    @Override
    public void initSendable(SendableBuilder builder) {}
}