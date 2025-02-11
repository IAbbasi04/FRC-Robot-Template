package frc.robot.helpers.motor;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.helpers.PIDProfile;

public class MinionMotor extends NewtonMotor {
    private TalonFXS motor;
    private TalonFXSConfiguration configuration;
    private TalonFXSConfigurator configurator;

    private DutyCycleOut percentPowerOutput = new DutyCycleOut(0);
    private VoltageOut voltageOutput = new VoltageOut(0);
    private MotionMagicVoltage motionMagicOutput = new MotionMagicVoltage(0);
    private PositionVoltage positionOutput = new PositionVoltage(0);
    private VelocityVoltage velocityOutput = new VelocityVoltage(0);

    public MinionMotor(int motorID) {
        this(motorID, false);
    }

    public MinionMotor(int motorID, boolean inverted) {
        super(
            motorID, 
            inverted, 
            new MotorConstants(
                7384, 
                200, 
                3.10, 
                627.6
            )
        );

        this.motor = new TalonFXS(motorID);
        this.configuration = new TalonFXSConfiguration();
        this.configurator = motor.getConfigurator();

        this.configuration.MotorOutput.Inverted = inverted ? 
            InvertedValue.Clockwise_Positive : 
            InvertedValue.CounterClockwise_Positive
        ;

        this.percentPowerOutput.OverrideBrakeDurNeutral = true;
        this.voltageOutput.OverrideBrakeDurNeutral = true;
        this.motionMagicOutput.OverrideBrakeDurNeutral = true;
        this.positionOutput.OverrideBrakeDurNeutral = true;
        this.velocityOutput.OverrideBrakeDurNeutral = true;
    }

    @Override
    public void setInverted(boolean inverted) {
        this.configuration.MotorOutput.Inverted = inverted ? 
            InvertedValue.Clockwise_Positive : 
            InvertedValue.CounterClockwise_Positive
        ;

        configurator.apply(configuration);
    }

    @Override
    public void withGains(PIDProfile gains) {
        super.motorPIDGains.add(gains.getSlot(), gains);
        
        switch (gains.pidSlot) {
            case 0:
                Slot0Configs slot0Config = configuration.Slot0
                    .withKP(gains.kP)
                    .withKI(gains.kI)
                    .withKD(gains.kD)
                    .withKA(gains.kA)
                    .withKV(gains.kV)
                    .withKS(gains.kS);

                this.motor.getConfigurator().apply(slot0Config);
            case 1:
                Slot1Configs slot1Config = configuration.Slot1
                    .withKP(gains.kP)
                    .withKI(gains.kI)
                    .withKD(gains.kD)
                    .withKA(gains.kA)
                    .withKV(gains.kV)
                    .withKS(gains.kS);

                this.motor.getConfigurator().apply(slot1Config);
                break;
            case 2:
                Slot2Configs slot2Config = configuration.Slot2
                    .withKP(gains.kP)
                    .withKI(gains.kI)
                    .withKD(gains.kD)
                    .withKA(gains.kA)
                    .withKV(gains.kV)
                    .withKS(gains.kS);

                this.motor.getConfigurator().apply(slot2Config);
                break;
            default:
                Slot0Configs slotConfig = configuration.Slot0
                    .withKP(gains.kP)
                    .withKI(gains.kI)
                    .withKD(gains.kD)
                    .withKA(gains.kA)
                    .withKV(gains.kV)
                    .withKS(gains.kS);

                this.motor.getConfigurator().apply(slotConfig);
                break;
        }
    }

    @Override
    public void configureMotionProfile(double maxVelocity, double maxAcceleration) {
        MotionMagicConfigs mmConfigs = configuration.MotionMagic;
        mmConfigs.withMotionMagicCruiseVelocity(maxVelocity);
        mmConfigs.withMotionMagicAcceleration(maxAcceleration);
        configurator.apply(configuration);
    }

    @Override
    public void setPercentOutput(double percent) {
        this.motor.setControl(percentPowerOutput.withOutput(percent));
    }

    @Override
    public void setVoltage(double voltage, int slot) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setVoltage'");
    }

    @Override
    public void setVelocity(double desiredRPM, int pidSlot) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setVelocity'");
    }

    @Override
    public void setPosition(double desiredRotations, int pidSlot) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPosition'");
    }

    @Override
    public void setFollowerTo(NewtonMotor master, boolean reversed) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setFollowerTo'");
    }

    @Override
    public void setCurrentLimit(int currentAmps) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setCurrentLimit'");
    }

    @Override
    public void setIdleMode(IdleMode idleMode) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setIdleMode'");
    }

    @Override
    public double getVelocityRPM() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getVelocityRPM'");
    }

    @Override
    public double getRotations() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getRotations'");
    }

    @Override
    public double getAppliedVoltage() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getAppliedVoltage'");
    }

    @Override
    public void resetEncoderPosition(double rotations) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetEncoderPosition'");
    }
}