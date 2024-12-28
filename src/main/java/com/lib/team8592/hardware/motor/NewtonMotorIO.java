package com.lib.team8592.hardware.motor;

import com.frc.robot.Robot;
import com.lib.team8592.PIDProfile;
import com.lib.team8592.hardware.NEW_NewtonPIDController;
// import com.lib.team8592.hardware.NewtonPIDController;
import com.lib.team8592.hardware.motor.spark.*;
import com.lib.team8592.hardware.motor.talonfx.*;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

    private double appliedVelocityRPM = 0d;

    private double dt = 0d;

    private NEW_NewtonPIDController newtonPID;

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

        this.newtonPID = new NEW_NewtonPIDController(pidGains);
    }

    public double fromRPMToVolts(double rpm) {
        return rpm / motors[0].getVoltageToRPMRatio() / gearRatio;
    }

    public double fromVoltsToRPM(double volts) {
        return volts * motors[0].getVoltageToRPMRatio() * gearRatio;
    }

    public void setVelocityRPM(double desiredRPM) {
        double newtonVelocity = newtonPID.calculate(
            this.getVelocityRPM(), 
            desiredRPM
        );
            
        this.desiredState = new State(0d, newtonVelocity);

        this.appliedVelocityRPM = this.calculate(
            new State(0d, motors[0].getVelocityRPM()), 
            desiredState
        );

        SmartDashboard.putNumber("LMAO/DESIRED VELOCITY ___ A", desiredState.velocity);
        SmartDashboard.putNumber("LMAO/APPLIED VELOCITY ___ A", appliedVelocityRPM);
        SmartDashboard.putNumber("LMAO/NEWTON VELOCITY ___ A", newtonVelocity);

        for (T motor : motors) {
            motor.setVelocity(appliedVelocityRPM);
        }

        double appliedVoltage = fromRPMToVolts(desiredRPM);
        gearboxSim.setInput(appliedVoltage);
    }

    public void update(double dt, boolean isSimulation) {
        this.dt = dt;
        if (isSimulation) {
            this.gearboxSim.update(dt);
            this.currentState = new State(
                0d, 
                gearboxSim.getAngularVelocityRPM()
            );
        } else {
            this.currentState = new State(motors[0].getRotations(), motors[0].getVelocityRPM());
        }
    }

    private double calculate(TrapezoidProfile.State current, TrapezoidProfile.State desired) {
        if (!lastDesiredState.equals(desired)) { // Goal has changed
            profileTimer.restart();
        }

        this.appliedVelocityRPM = pidCtrl.calculate(
            current.velocity, 
            trapezoidProfile.calculate(
                dt,
                current, 
                desired
            ).velocity
        );

        this.lastDesiredState = desired;
        return appliedVelocityRPM;
    }

    public double getRotations() {
        if (Robot.isSimulation()) {
            return this.gearboxSim.getAngularPositionRotations();
        }
        return this.motors[0].getRotations();
    }

    public double getVelocityRPM() {
        if (Robot.isSimulation()) {
            return this.gearboxSim.getAngularVelocityRPM();
        }
        return this.motors[0].getVelocityRPM();
    }

    public double getAppliedVelocityRPM() {
        return this.appliedVelocityRPM;
    }
}