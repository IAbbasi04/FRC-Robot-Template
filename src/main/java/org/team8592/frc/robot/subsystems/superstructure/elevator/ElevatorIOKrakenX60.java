package org.team8592.frc.robot.subsystems.superstructure.elevator;

import org.team8592.lib.hardware.motor.talonfx.KrakenX60Motor;

public class ElevatorIOKrakenX60 extends ElevatorIO {
    private KrakenX60Motor backElevatorMotor;
    private KrakenX60Motor frontElevatorMotor;

    public ElevatorIOKrakenX60(int CANIdFront, boolean reversedFront, int CANIdBack, boolean reversedBack) {
        this.frontElevatorMotor = new KrakenX60Motor(CANIdFront, reversedFront);

        this.backElevatorMotor = new KrakenX60Motor(CANIdBack, reversedBack);
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