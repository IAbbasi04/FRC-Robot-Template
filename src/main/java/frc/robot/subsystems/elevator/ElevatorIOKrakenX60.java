package frc.robot.subsystems.elevator;

import lib.team8592.hardware.motor.talonfx.TalonFXMotor;

public class ElevatorIOKrakenX60 extends ElevatorIO {
    private TalonFXMotor elevatorMotor;

    public ElevatorIOKrakenX60(int motorID, boolean inverted) {
        this.elevatorMotor = new TalonFXMotor(motorID, inverted);
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

    @Override
    public void updateInputs() {}
}