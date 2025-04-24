package org.team8592.frc.robot.subsystems.superstructure.elevator;

import org.team8592.lib.hardware.motor.talonfx.TalonFXMotor;

public class ElevatorIOKrakenX60 extends ElevatorIO {
    private TalonFXMotor backElevatorMotor;
    private TalonFXMotor frontElevatorMotor;

    public ElevatorIOKrakenX60(int CANIdFront, boolean reversedFront, int CANIdBack, boolean reversedBack) {
        this.frontElevatorMotor = new TalonFXMotor(CANIdFront, reversedFront);

        this.backElevatorMotor = new TalonFXMotor(CANIdBack, reversedBack);
        this.backElevatorMotor.setFollowerTo(frontElevatorMotor, reversedBack);
    }

    @Override
    public void setPercentOutput(double percentOutput) {
        this.frontElevatorMotor.setPercentOutput(percentOutput);
    }

    @Override
    public void setInches(double inches) {
        this.frontElevatorMotor.setPosition(inches);
    }

    @Override
    public double getInches() {
        return this.frontElevatorMotor.getRotations();
    }
}