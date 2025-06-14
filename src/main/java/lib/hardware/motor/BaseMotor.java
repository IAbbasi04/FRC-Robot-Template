package lib.hardware.motor;

import java.util.ArrayList;
import java.util.List;

import lib.PIDProfile;
import lib.hardware.motor.ctre.TalonFXMotor;
import lib.hardware.motor.ctre.TalonFXSMotor;
import lib.hardware.motor.rev.*;
import edu.wpi.first.wpilibj.simulation.*;

public abstract class BaseMotor {
    protected List<PIDProfile> motorPIDGains = new ArrayList<>();
    protected String canBusName = "";
    protected int deviceID = 0;
    protected boolean inverted = false;
    protected double desiredVelocityRPM = 0d;
    protected EncoderSim simEncoder; 
    protected DCMotorSim simMotor;

    protected BaseMotor(PortConfig config) {
        this.canBusName = config.kBus;
        this.deviceID = config.kPort;
        this.inverted = config.kInverted;
    }

    public enum IdleMode {
        kBrake,
        kCoast
    }

    public abstract void setInverted(boolean inverted);

    public abstract void withGains(PIDProfile gains);

    public void withGains(PIDProfile... gains) {
        for (PIDProfile gain : gains) {
            this.withGains(gain);
        }
    }

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
    
    public abstract void setFollowerTo(BaseMotor master, boolean reversed);
    
    public void setFollowerTo(BaseMotor master) {
        setFollowerTo(master, false);
    }

    public abstract void setCurrentLimit(int currentAmps);

    public abstract void setIdleMode(IdleMode idleMode);

    public abstract double getVelocityRPM();

    public abstract double getRotations();

    public abstract double getAppliedVoltage();

    public abstract void resetEncoderPosition(double rotations);

    public abstract void setSoftLimits(double min, double max);

    public abstract void configureMotionProfile(double maxVelocity, double maxAcceleration);

    public abstract double getVoltage();

    public boolean isInverted() {
        return this.inverted;
    }

    public int getDeviceID() {
        return this.deviceID;
    }

    public List<PIDProfile> getPIDGains() {
        if (motorPIDGains.size() == 0) this.motorPIDGains.add(0, new PIDProfile());
        return this.motorPIDGains;
    }

    public double getDesiredVelocity() {
        return this.desiredVelocityRPM;
    }

    public SparkFlexMotor getAsSparkFlex() {
        return getAsMotorType(SparkFlexMotor.class);
    }

    public SparkMaxMotor getAsSparkMax() {
        return getAsMotorType(SparkMaxMotor.class);
    }

    public TalonFXMotor getAsTalonFX() {
        return getAsMotorType(TalonFXMotor.class);
    }

    public TalonFXSMotor getAsTalonFXS() {
        return getAsMotorType(TalonFXSMotor.class);
    }

    @SuppressWarnings("unchecked")
    private <M extends BaseMotor> M getAsMotorType(Class<M> cls) {
        return (M)this;
    }
}