package com.lib.team8592.hardware.motor;

import com.frc.robot.Robot;
import com.lib.team8592.PIDProfile;
import com.lib.team8592.hardware.NEW_NewtonPIDController;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NewtonMotorIO<T extends NewtonMotor> {
    private TrapezoidProfile trapezoidProfile = null;
    private PIDProfile pidGains = null;
    private T[] motors = null;

    private Timer profileTimer = new Timer();

    private DCMotorSim gearboxSim;

    private double gearRatio = 1d;

    private State goalState = new State();

    private double appliedVelocityRPM = 0d;

    private NEW_NewtonPIDController newtonPID;

    @SuppressWarnings("unchecked")
    public NewtonMotorIO(PIDProfile pidGains, double gearRatio, double momentOfInertia, T ... motors) {
        this.pidGains = pidGains;
        this.motors = motors;
        this.gearRatio = gearRatio;
        this.newtonPID = new NEW_NewtonPIDController(pidGains);

        if (motors.length == 0) {
            throw new UnsupportedOperationException("No motors in list!");
        }

        this.gearboxSim = new DCMotorSim(
            NewtonMotor.getDCMotor(motors[0], motors.length), 
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
        double newtonVelocity = newtonPID.calculate(
            this.getVelocityRPM(), 
            desiredRPM
        );
            
        this.goalState = new State(0d, newtonVelocity);

        SmartDashboard.putNumber("NewtonMotorIO/Goal Velocity", goalState.velocity);
        SmartDashboard.putNumber("NewtonMotorIO/Newton Velocity", newtonVelocity);
        SmartDashboard.putNumber("NewtonMotorIO/Current RPM", getVelocityRPM());
        SmartDashboard.putNumber("NewtonMotorIO/Current Rotations", getRotations());

        for (T motor : motors) {
            motor.setVelocity(appliedVelocityRPM);
        }

        double appliedVoltage = fromRPMToVolts(desiredRPM);
        gearboxSim.setInputVoltage(appliedVoltage);
    }

    public void update(double dt, boolean isSimulation) {
        if (isSimulation) {
            this.gearboxSim.update(dt);
        }
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