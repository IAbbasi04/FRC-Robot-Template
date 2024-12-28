package com.lib.team8592.hardware.motor;

import java.util.ArrayList;
import java.util.List;

import com.lib.team8592.PIDProfile;
import com.lib.team8592.hardware.NewtonFeedForward;
import com.lib.team8592.hardware.motor.spark.SparkFlexMotor;
import com.lib.team8592.hardware.motor.spark.SparkMaxMotor;
import com.lib.team8592.hardware.motor.talonfx.TalonFXMotor;

public abstract class NewtonMotor {
    protected List<PIDProfile> motorPIDGains = new ArrayList<>();
    protected List<NewtonFeedForward> feedForward = new ArrayList<>();
    protected int deviceID = 0;
    protected boolean inverted = false;
    protected MotorConstants motorConstants = null;
    protected double desiredVelocityRPM = 0d;

    protected NewtonMotor(int id, boolean inverted, MotorConstants constants) {
        this.deviceID = id;
        this.inverted = inverted;
        this.motorConstants = constants;
    }

    public enum IdleMode {
        kBrake,
        kCoast
    }

    public abstract void setInverted(boolean inverted);

    public abstract void withGains(PIDProfile gains);
    
    public abstract void setPercentOutput(double percent);

    public abstract void setVelocity(double desiredRPM, int pidSlot);

    public void setVelocity(double desiredRPM) {
        setVelocity(desiredRPM, 0);
    }

    public abstract void setPositionSmartMotion(double desiredRotations, int pidSlot);

    public void setPositionSmartMotion(double desiredRotations) {
        setPositionSmartMotion(desiredRotations, 0);
    }
    
    public abstract void setFollowerTo(NewtonMotor master, boolean reversed);
    
    public void setFollowerTo(NewtonMotor master) {
        setFollowerTo(master, false);
    }

    public abstract void setCurrentLimit(int currentAmps);

    public abstract void setIdleMode(IdleMode idleMode);

    public abstract double getVelocityRPM();

    public abstract double getRotations();

    public abstract double getAppliedVoltage();

    public abstract void resetEncoderPosition(double rotations);

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