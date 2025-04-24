package org.team8592.lib.hardware.motor;

import java.util.ArrayList;
import java.util.List;

import org.team8592.lib.PIDProfile;
import org.team8592.lib.hardware.motor.spark.*;
import org.team8592.lib.hardware.motor.talonfx.*;

import edu.wpi.first.wpilibj.simulation.*;

public abstract class NewtonMotor {
    protected List<PIDProfile> motorPIDGains = new ArrayList<>();
    protected int deviceID = 0;
    protected boolean inverted = false;
    protected MotorConstants motorConstants = null;
    protected double desiredVelocityRPM = 0d;
    protected EncoderSim simEncoder; 
    protected DCMotorSim simMotor;

    protected NewtonMotor(int id, boolean inverted) {
        this.deviceID = id;
        this.inverted = inverted;
    }

    public enum IdleMode {
        kBrake,
        kCoast
    }

    public abstract void setInverted(boolean inverted);

    public abstract void withGains(PIDProfile gains);

    public void withGains(PIDProfile gains, int slot) {
        this.withGains(gains.setSlot(slot));
    }
    
    public abstract void setPercentOutput(double percent);

    public abstract void setVoltage(double voltage, int slot);

    public void setVoltage(double voltage) {
        setVoltage(voltage, 0);
    }

    public abstract void setVelocity(double desiredRPM, int pidSlot);

    public void setVelocity(double desiredRPM) {
        setVelocity(desiredRPM, 0);
    }

    public abstract void setPosition(double desiredRotations, int pidSlot);

    public void setPosition(double desiredRotations) {
        setPosition(desiredRotations, 0);
    }
    
    public abstract void setFollowerTo(NewtonMotor master, boolean reversed);
    
    public void setFollowerTo(NewtonMotor master) {
        setFollowerTo(master, false);
    }

    public abstract void setCurrentLimit(int currentAmps);

    public abstract void setIdleMode(IdleMode idleMode);

    public abstract double getVelocityRPM();

    public abstract double getRotations();

    public abstract double getInputVoltage();

    public abstract double getMotorVoltage();

    public abstract void resetEncoderPosition(double rotations);

    public abstract void setSoftLimits(double min, double max);

    public abstract void configureMotionProfile(double maxVelocity, double maxAcceleration);

    public boolean isInverted() {
        return this.inverted;
    }

    public int getDeviceID() {
        return this.deviceID;
    }

    public double getMaxFreeVelocity() {
        return this.motorConstants.MAX_VELOCITY_RPM;
    }

    public MotorConstants getMotorConstants() {
        return this.motorConstants;
    }

    public double getVoltageToRPMRatio() {
        return this.motorConstants.MOTOR_KV;
    }

    public List<PIDProfile> getPIDGains() {
        if (motorPIDGains.size() == 0) this.motorPIDGains.add(0, new PIDProfile());
        return this.motorPIDGains;
    }

    public double getDesiredVelocity() {
        return this.desiredVelocityRPM;
    }

    public SparkFlexMotor getAsSparkFlex() {
        return (SparkFlexMotor)this;
    }

    public SparkMaxMotor getAsSparkMax() {
        return (SparkMaxMotor)this;
    }

    public TalonFXMotor getAsTalonFX() {
        return (TalonFXMotor)this;
    }
}