package com.lib.team8592.hardware;

import com.lib.team8592.PIDProfile;
import com.lib.team8592.hardware.motor.NewtonMotor;
import com.lib.team8592.hardware.motor.spark.*;
import com.lib.team8592.hardware.motor.talonfx.*;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class NewtonMotorIO<T extends NewtonMotor> {
    private ProfiledPIDController pidCtrl = null;
    private TrapezoidProfile trapezoidProfile = null;
    private PIDProfile pidGains = null;
    private T[] motors = null;

    private Timer profileTimer = new Timer();

    private DCMotorSim gearboxSim;

    private double gearRatio = 1d;

    private State lastDesiredState = new State();
    private State currentState = new State();
    private State desiredState = new State();

    // private double profileOutput = 0d;

    @SuppressWarnings("unchecked")
    public NewtonMotorIO(PIDProfile pidGains, double gearRatio, double momentOfInertia, T ... motors) {
        this.pidGains = pidGains;
        this.pidCtrl = this.pidGains.toProfiledPIDController();
        this.motors = motors;

        this.gearRatio = gearRatio;

        if (motors.length == 0) {
            throw new UnsupportedOperationException("No motors in list!");
        }

        DCMotor gearbox = null;
        if (motors[0].getClass().equals(SparkFlexMotor.class)) { // Vortex Motor
            gearbox = DCMotor.getNeoVortex(motors.length);
        } else if (motors[0].getClass().equals(SparkMaxMotor.class)) { // Neo Motor
            gearbox = DCMotor.getNEO(motors.length);
        } else if (motors[0].getClass().equals(Falcon500Motor.class)) {
            gearbox = DCMotor.getFalcon500(motors.length);
        } else if (motors[0].getClass().equals(Falcon500FOCMotor.class)) {
            gearbox = DCMotor.getFalcon500Foc(motors.length);
        } else if (motors[0].getClass().equals(KrakenX60Motor.class)) {
            gearbox = DCMotor.getKrakenX60(motors.length);
        } else if (motors[0].getClass().equals(KrakenX60FOCMotor.class)) {
            gearbox = DCMotor.getKrakenX60Foc(motors.length);
        }

        this.gearboxSim = new DCMotorSim(
            gearbox, 
            gearRatio, 
            momentOfInertia
        );

        this.trapezoidProfile = new TrapezoidProfile(new Constraints(
            pidGains.maxVelocity, 
            pidGains.maxAcceleration
        ));
    }

    public double fromRPMToVolts(double rpm) {
        return rpm / motors[0].getVoltageToRPMRatio() / gearRatio;
    }

    public double fromVoltsToRPM(double volts) {
        return volts * motors[0].getVoltageToRPMRatio() * gearRatio;
    }

    public void setVelocityRPM(double desiredRPM) {
        for (T motor : motors) {
            motor.setVelocity(desiredRPM);
        }

        double appliedVoltage = fromRPMToVolts(desiredRPM);
        gearboxSim.setInput(appliedVoltage);
    }

    public void updateSim(double dt) {
        this.gearboxSim.update(dt);
    }

    // public void calculate(TrapezoidProfile.State current, TrapezoidProfile.State desired) {
    //     if (!lastDesiredState.equals(desired)) { // Goal has changed
    //         profileTimer.restart();
    //     }

    //     // this.profileOutput = trapezoidProfile.calculate(profileTimer.get(), current, desired).velocity;
    //     this.lastDesiredState = desired;

    //     // double applied = pidCtrl.calculate(current.velocity, profileOutput);
    // }

    // public boolean atReference() {
    //     return pidCtrl.atSetpoint();
    // }
}