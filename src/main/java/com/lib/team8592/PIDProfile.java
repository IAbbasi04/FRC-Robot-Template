package com.lib.team8592;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.sendable.*;
import com.lib.team8592.logging.SmartLogger;

public class PIDProfile implements Sendable {
    private SmartLogger gainsLogger;

    public int pidSlot = 0;

    public double kP = 0;
    public double kI = 0;
    public double kD = 0;
    public double kFF = 0;
    public double kS = 0;
    public double kA = 0;
    public double kV = 0;
    public double kG = 0;

    public double maxAcceleration = Double.POSITIVE_INFINITY;
    public double maxVelocity = Double.POSITIVE_INFINITY;

    public double tolerance = 0;

    public boolean continuousInput = false;
    public double continuousMin = Double.NEGATIVE_INFINITY;
    public double continuousMax = Double.POSITIVE_INFINITY;

    public boolean softLimit = false;
    public double softLimitMin = Double.NEGATIVE_INFINITY;
    public double softLimitMax = Double.POSITIVE_INFINITY;

    public double scale = 1.0;

    public int currentLimit = -1;

    public boolean useSmartMotion = false;

    public PIDProfile() {
        SendableRegistry.add(this, "PIDGainsProfile", 1);
    }

    public PIDProfile setP(double gain) {
        kP = gain;
        return this;
    }

    public double getP() {
        return kP;
    }

    public PIDProfile setI(double gain) {
        kI = gain;
        return this;
    }

    public double getI() {
        return kI;
    }

    public PIDProfile setD(double gain) {
        kD = gain;
        return this;
    }

    public double getD() {
        return kD;
    }

    public PIDProfile setFF(double gain) {
        kFF = gain;
        return this;
    }

    public double getFF() {
        return kFF;
    }

    public PIDProfile setV(double gain) {
        this.kV = gain;
        return this;
    }

    public double getV() {
        return kV;
    }

    public PIDProfile setA(double gain) {
        this.kA = gain;
        return this;
    }

    public double getA() {
        return kA;
    }

    public PIDProfile setS(double gain) {
        this.kS = gain;
        return this;
    }

    public double getS() {
        return kS;
    }

    public PIDProfile setG(double gain) {
        this.kG = gain;
        return this;
    }

    public double getG() {
        return kG;
    }

    public PIDProfile setMaxAcceleration(double gain) {
        maxAcceleration = gain;
        useSmartMotion = true;
        return this;
    }

    public double getMaxAcceleration() {
        return maxAcceleration;
    }

    public PIDProfile setMaxVelocity(double gain) {
        maxVelocity = gain;
        useSmartMotion = true;
        return this;
    }

    public double getMaxVelocity() {
        return maxVelocity;
    }

    public PIDProfile setSlot(int slot) {
        pidSlot = slot;
        return this;
    }

    public int getSlot() {
        return pidSlot;
    }

    public PIDProfile setTolerance(double tolerance) {
        this.tolerance = tolerance;
        return this;
    }

    public double getTolerance() {
        return tolerance;
    }

    /**
     * Useful for continually rotating mechanisms
     */
    public PIDProfile setContinuousInput(double min, double max) {
        this.continuousMin = min;
        this.continuousMax = max;
        this.continuousInput = true;
        return this;
    }

    public boolean isContinuousInput() {
        return continuousInput;
    }

    /**
     * Currently only works for neo motors
     */
    public PIDProfile setSoftLimits(double min, double max) {
        this.softLimitMin = min;
        this.softLimitMax = max;
        this.softLimit = true;
        return this;
    }

    public boolean hasSoftLimits() {
        return softLimit;
    }

    /**
     * Double array returns both min and max -> [minimum input, maximum input]
     */
    public float[] getSoftLimits() {
        return new float[] {(float)softLimitMin, (float)softLimitMax};
    }

    /**
     * Sets a scaling factor for the output
     */
    public PIDProfile setScalingFactor(double scale) {
        this.scale = scale;
        return this;
    }

    /**
     * Returns the scaling factor
     */
    public double getScalingFactor() {
        return scale;
    }

    /**
     * Sets a maximum current limit
     */
    public PIDProfile setCurrentLimit(int limit) {
        this.currentLimit = limit;
        return this;
    }

    /**
     * Returns the maximum current limit
     */
    public int getCurrentLimit() {
        return currentLimit;
    }

    /**
     * If smart motion and/or smart velocity should be applied
     */
    public boolean isSmartMotion() {
        return useSmartMotion;
    }

    /**
     * Creates a PID controller with the specified constants and configurations
     */
    public ProfiledPIDController toProfiledPIDController() {
        ProfiledPIDController pidCtrl = new ProfiledPIDController(kP, kI, kD, new Constraints(maxVelocity * scale, maxAcceleration * scale));
        pidCtrl.setTolerance(tolerance);
        if (continuousInput) {
            pidCtrl.enableContinuousInput(continuousMin, continuousMax);
        }
        return pidCtrl;
    }

    /**
     * Creates a PID controller with the specified constants and configurations
     */
    public PIDController toPIDController() {
        PIDController pidCtrl = new PIDController(kP, kI, kD);
        pidCtrl.setTolerance(tolerance);
        if (continuousInput) {
            pidCtrl.enableContinuousInput(continuousMin, continuousMax);
        }
        return pidCtrl;
    }

    public void logAsPIDController(String name) {
        if (this.gainsLogger != null) return;

        this.gainsLogger = new SmartLogger(name).initialize();

        this.gainsLogger.log("kP", kP);
        this.gainsLogger.log("kI", kI);
        this.gainsLogger.log("kD", kD);
    }

    public void logAsProfiledPIDController(String name) {
        this.gainsLogger = new SmartLogger(name).initialize();

        this.gainsLogger.log("kP", kP);
        this.gainsLogger.log("kI", kI);
        this.gainsLogger.log("kD", kD);
        this.gainsLogger.log("kFF", kFF);
        this.gainsLogger.log("kA", kA);
        this.gainsLogger.log("kV", kV);
        this.gainsLogger.log("kS", kS);
        this.gainsLogger.log("kG", kG);
        this.gainsLogger.log("MaxVelocity", maxVelocity);
        this.gainsLogger.log("MaxAcceleration", maxAcceleration);
    }

    public PIDProfile fromLoggedProfiledPIDController() {
        this.setP(gainsLogger.getEntry("kP").getDouble(kP));
        this.setI(gainsLogger.getEntry("kI").getDouble(kI));
        this.setD(gainsLogger.getEntry("kD").getDouble(kD));
        this.setG(gainsLogger.getEntry("kG").getDouble(kG));
        this.setA(gainsLogger.getEntry("kA").getDouble(kA));
        this.setS(gainsLogger.getEntry("kS").getDouble(kS));
        this.setV(gainsLogger.getEntry("kV").getDouble(kV));
        this.setFF(gainsLogger.getEntry("kFF").getDouble(kFF));
        this.setMaxVelocity(gainsLogger.getEntry("MaxVelocity").getDouble(maxVelocity));
        this.setMaxAcceleration(gainsLogger.getEntry("MaxAcceleration").getDouble(maxAcceleration));
        return this;
    }

    public PIDProfile fromLoggedPIDController() {
        this.setP(gainsLogger.getEntry("kP").getDouble(kP));
        this.setI(gainsLogger.getEntry("kI").getDouble(kI));
        this.setD(gainsLogger.getEntry("kD").getDouble(kD));
        return this;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("PIDGainsProfile");
        builder.addDoubleProperty("p", this::getP, this::setP);
        builder.addDoubleProperty("i", this::getI, this::setI);
        builder.addDoubleProperty("d", this::getD, this::setD);
        builder.addDoubleProperty("f", this::getFF, this::setFF);
        builder.addDoubleProperty("s", this::getS, this::setS);
        builder.addDoubleProperty("a", this::getA, this::setA);
        builder.addDoubleProperty("v", this::getV, this::setV);
        builder.addDoubleProperty("g", this::getG, this::setG);
        builder.addDoubleProperty("maxVelocity", this::getMaxVelocity, this::setMaxVelocity);
        builder.addDoubleProperty("maxAccel", this::getMaxAcceleration, this::setMaxAcceleration);
    }
}