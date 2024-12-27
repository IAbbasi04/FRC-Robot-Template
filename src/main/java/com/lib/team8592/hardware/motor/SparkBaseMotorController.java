package com.lib.team8592.hardware.motor;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.lib.team8592.PIDProfile;
import com.lib.team8592.Utils;
import com.revrobotics.CANSparkBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.AccelStrategy;

public class SparkBaseMotorController<M extends CANSparkBase> extends MotorController {
    protected M motor; // Made public so it can be accessed as a follower
    protected SparkPIDController motorCtrl;
    protected RelativeEncoder encoder;

    protected SparkBaseMotorController(M motor, boolean reversed) {
        super(motor.getDeviceId(), reversed);
        this.motor = motor;
        this.motorCtrl = motor.getPIDController();
        this.encoder = motor.getEncoder();
        this.motor.setInverted(reversed);
    }

    @Override
    public void setInverted(boolean inverted) {
        this.motor.setInverted(inverted);
    }

    @Override
    public void withGains(PIDProfile gains) {
        super.motorPIDGains.add(gains.getSlot(), gains);
        
        this.motorCtrl.setP(gains.kP);
        this.motorCtrl.setI(gains.kI);
        this.motorCtrl.setD(gains.kD);
        // this.motorCtrl.setFF(gains.kFF);

        if (gains.softLimit) { // If soft limit values applied in the gains profile
            this.motor.enableSoftLimit(SoftLimitDirection.kForward, true);
            this.motor.enableSoftLimit(SoftLimitDirection.kReverse, true);

            this.motor.setSoftLimit(SoftLimitDirection.kForward, (float)gains.softLimitMax);
            this.motor.setSoftLimit(SoftLimitDirection.kReverse, (float)gains.softLimitMin);
        }

        this.motorCtrl.setSmartMotionAllowedClosedLoopError(gains.getTolerance(), gains.getSlot());
        this.motorCtrl.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, gains.getSlot());
        this.motorCtrl.setSmartMotionMaxVelocity(gains.getMaxVelocity(), gains.getSlot());
        this.motorCtrl.setSmartMotionMaxAccel(gains.getMaxAcceleration(), gains.getSlot());
    }

    @Override
    public void setPercentOutput(double percent) {
        this.motor.set(percent);
    }

    @Override
    public void setVelocity(double desiredVelocityRPM, int pidSlot) {
        if (motorPIDGains != null) {
            Utils.clamp(
                desiredVelocityRPM, 
                -motorPIDGains.get(pidSlot).maxVelocity,
                motorPIDGains.get(pidSlot).maxVelocity
            );
        }
        this.motorCtrl.setReference(desiredVelocityRPM, ControlType.kSmartVelocity);
    }

    @Override
    public void setPositionSmartMotion(double desiredRotations, int pidSlot) {
        if (motorPIDGains != null) {
            Utils.clamp(
                desiredRotations, 
                motorPIDGains.get(pidSlot).softLimitMin,
                motorPIDGains.get(pidSlot).softLimitMax
            );
        }
        this.motorCtrl.setReference(desiredRotations, ControlType.kSmartMotion);
    }

    @Override
    public void setFollowerTo(MotorController master, boolean reversed) {
        motor.follow(master.getAsSparkFlex().motor, reversed);
    }

    @Override
    public void setCurrentLimit(int currentAmps) {
        this.motor.setSmartCurrentLimit(currentAmps);
        this.motor.setSecondaryCurrentLimit(currentAmps);
    }

    @Override
    public void setIdleMode(IdleMode idleMode) {
        switch (idleMode) {
            case kCoast:
                motor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kCoast);
                break;
            case kBrake:
            // On default set to brake mode
            default:
                motor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kBrake);
                break;
        }
    }

    @Override
    public double getVelocityRPM() {
        return encoder.getVelocity();
    }

    @Override
    public double getRotations() {
        return encoder.getPosition();
    }

    @Override
    public void resetEncoderPosition(double rotations) {
        this.encoder.setPosition(rotations);
    }
}