package org.team8592.frc.robot.subsystems.superstructure.wrist;

import org.team8592.frc.robot.Constants;
import org.team8592.lib.PIDProfile;
import org.team8592.lib.hardware.motor.talonfx.TalonFXMotor;

public class WristIOKrakenX60 extends WristIO {
    private TalonFXMotor wristMotor;
    private PIDProfile positionGains = new PIDProfile().setP(3.0);
    
    public WristIOKrakenX60(int CANId, boolean reversed) {
        this.wristMotor = new TalonFXMotor(CANId, reversed);
        this.wristMotor.withGains(positionGains);
        this.wristMotor.configureMotionProfile(
            Constants.WRIST.WRIST_MAX_VELOCITY, 
            Constants.WRIST.WRIST_MAX_ACCELERATION
        );

        this.wristMotor.setCurrentLimit(Constants.WRIST.WRIST_CURRENT_LIMIT);
    }

    public WristIOKrakenX60(int CANId) {
        this(CANId, false);
    }

    @Override
    public void setPercentOutput(double desiredPercent) {
        this.wristMotor.setPercentOutput(desiredPercent);
    }

    @Override
    public void setDegrees(double degrees) {
        this.wristMotor.setPosition(degrees);
    }

    @Override
    public double getDegrees() {
        return wristMotor.getRotations() * Constants.WRIST.WRIST_GEAR_RATIO;
    }
}