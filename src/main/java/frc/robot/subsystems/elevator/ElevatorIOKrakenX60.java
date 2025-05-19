package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.GEAR_RATIO;

import lib.team8592.hardware.motor.talonfx.TalonFXMotor;

public class ElevatorIOKrakenX60 extends ElevatorIO {
    private TalonFXMotor elevatorMotor;

    public ElevatorIOKrakenX60(int motorID, boolean inverted) {
        this.elevatorMotor = new TalonFXMotor(motorID, inverted);
    }

    private double getMotorRotationsFromInches(double inches) {
        return inches * GEAR_RATIO;
    }

    private double getInchesFromMotorRotations(double rotations) {
        return rotations / GEAR_RATIO;
    }

    @Override
    public void setInches(double inches) {
        this.desiredInches = inches;
        this.elevatorMotor.setPosition(getMotorRotationsFromInches(inches));
    }

    @Override
    public double getCurrentInches() {
        return getInchesFromMotorRotations(elevatorMotor.getRotations());
    }

    @Override
    public void halt() {
        elevatorMotor.setPosition(getMotorRotationsFromInches(getCurrentInches()));
    }

    @Override
    public void updateInputs() {}
}