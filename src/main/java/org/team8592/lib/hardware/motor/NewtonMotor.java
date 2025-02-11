package org.team8592.lib.hardware.motor;

import java.util.ArrayList;
import java.util.List;

import org.team8592.lib.PIDProfile;
import org.team8592.lib.hardware.NewtonFeedForward;
import org.team8592.lib.hardware.motor.spark.SparkFlexMotor;
import org.team8592.lib.hardware.motor.spark.SparkMaxMotor;
import org.team8592.lib.hardware.motor.talonfx.Falcon500FOCMotor;
import org.team8592.lib.hardware.motor.talonfx.Falcon500Motor;
import org.team8592.lib.hardware.motor.talonfx.KrakenX60FOCMotor;
import org.team8592.lib.hardware.motor.talonfx.KrakenX60Motor;
import org.team8592.lib.hardware.motor.talonfx.TalonFXMotor;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

public abstract class NewtonMotor {
    protected List<PIDProfile> motorPIDGains = new ArrayList<>();
    protected List<NewtonFeedForward> feedForward = new ArrayList<>();
    protected int deviceID = 0;
    protected boolean inverted = false;
    protected MotorConstants motorConstants = null;
    protected double desiredVelocityRPM = 0d;
    protected EncoderSim simEncoder; 
    protected DCMotorSim simMotor;

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

    public abstract void setVoltage(double voltage, int slot);

    public void setVoltage(double voltage) {
        setVoltage(voltage, 0);
    }

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

    public static <M extends NewtonMotor> DCMotor getDCMotor(M motor, int numMotors) {
        DCMotor dcMotor = null;
        if (motor.getClass().equals(SparkFlexMotor.class)) { // Vortex Motor
            dcMotor = DCMotor.getNeoVortex(numMotors);
        } else if (motor.getClass().equals(SparkMaxMotor.class)) { // Neo Motor
            dcMotor = DCMotor.getNEO(numMotors);
        } else if (motor.getClass().equals(Falcon500Motor.class)) {
            dcMotor = DCMotor.getFalcon500(numMotors);
        } else if (motor.getClass().equals(Falcon500FOCMotor.class)) {
            dcMotor = DCMotor.getFalcon500Foc(numMotors);
        } else if (motor.getClass().equals(KrakenX60Motor.class)) {
            dcMotor = DCMotor.getKrakenX60(numMotors);
        } else if (motor.getClass().equals(KrakenX60FOCMotor.class)) {
            dcMotor = DCMotor.getKrakenX60Foc(numMotors);
        }
        return dcMotor;
    }

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