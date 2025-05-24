package frc.robot.subsystems.elevator;

import lib.team8592.hardware.motor.*;

public class ElevatorIOKrakenX60 extends ElevatorIO {
    private TalonFXMotor elevatorMotor;

    public ElevatorIOKrakenX60(PortConfig config) {
        this.elevatorMotor = new TalonFXMotor(config);
    }

    @Override
    public void setPosition(double inches) {
        this.desiredPosition = inches;
        this.elevatorMotor.setPosition(getMotorRotationsFromInches(inches));
    }

    @Override
    public double getPosition() {
        return getInchesFromMotorRotations(elevatorMotor.getRotations());
    }
}