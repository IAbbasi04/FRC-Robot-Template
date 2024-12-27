package com.lib.team8592.hardware.motor;

import java.util.ArrayList;
import java.util.List;

import com.lib.team8592.PIDProfile;
import com.lib.team8592.hardware.NewtonFeedForward;

public abstract class MotorController {
    protected List<PIDProfile> motorPIDGains = new ArrayList<>();
    protected List<NewtonFeedForward> feedForward = new ArrayList<>();
    protected int deviceID = 0;
    protected boolean reversed = false;

    protected MotorController(int id) {
        this(id, false);
    }

    protected MotorController(int id, boolean reversed) {
        this.deviceID = id;
        this.reversed = reversed;
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
    
    public abstract void setFollowerTo(MotorController master, boolean reversed);
    
    public void setFollowerTo(MotorController master) {
        setFollowerTo(master, false);
    }

    public abstract void setCurrentLimit(int currentAmps);

    public abstract void setIdleMode(IdleMode idleMode);

    public abstract double getVelocityRPM();

    public abstract double getRotations();

    public abstract void resetEncoderPosition(double rotations);

    public boolean isReversed() {
        return this.reversed;
    }

    public int getDeviceID() {
        return this.deviceID;
    }

    public SparkFlexMotorController getAsSparkFlex() {
        return (SparkFlexMotorController)this;
    }

    public SparkMaxMotorController getAsSparkMax() {
        return (SparkMaxMotorController)this;
    }

    public TalonFXMotorController getAsTalonFX() {
        return (TalonFXMotorController)this;
    }
}