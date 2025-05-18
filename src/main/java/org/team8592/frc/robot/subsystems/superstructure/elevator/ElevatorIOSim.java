package org.team8592.frc.robot.subsystems.superstructure.elevator;

import org.team8592.frc.robot.Robot;
import org.team8592.frc.robot.Constants.ELEVATOR;
import org.team8592.frc.robot.Constants.SUPERSTRUCTURE;
import org.team8592.frc.robot.subsystems.superstructure.Superstructure;
import org.team8592.lib.PIDProfile;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim extends ElevatorIO {
    private final ProfiledPIDController positionCtrl = new PIDProfile()
        .setP(1.0)
        .setMaxVelocity(100d)
        .setMaxAcceleration(300d)
        .toProfiledPIDController();

    private ElevatorSim elevatorSim = new ElevatorSim(
        DCMotor.getKrakenX60(2),
        1d / ELEVATOR.EXTENSION_GEAR_RATIO,
        SUPERSTRUCTURE.ELEVATOR_CARRIAGE_MASS_KG,
        ELEVATOR.EXTENSION_DRUM_DIAMETER_INCHES,
        SUPERSTRUCTURE.ELEVATOR_LENGTH_METERS,
        SUPERSTRUCTURE.ELEVATOR_LENGTH_METERS + Units.inchesToMeters(ELEVATOR.EXTENSION_INCHES_MAX),
        false,
        SUPERSTRUCTURE.ELEVATOR_LENGTH_METERS
    );

    @Override
    public void setInches(double inches) {
        double desiredVoltage = positionCtrl.calculate(getInches(), inches);
        this.elevatorSim.setInput(0, desiredVoltage, 0);
    }

    @Override
    public double getInches() {
        return Units.metersToInches(elevatorSim.getPositionMeters());
    }

    @Override
    public void setPercentOutput(double percentOutput) {
        this.elevatorSim.setInput(percentOutput * 12d);
    }

    @Override
    public void updateInputs() {
        this.elevatorSim.update(Robot.CLOCK.dt());
        // Superstructure.elevatorLigament.setLength(Units.inchesToMeters(getInches()) + SUPERSTRUCTURE.ELEVATOR_LENGTH_METERS);
        Superstructure.elevatorLigament.setAngle(Robot.CLOCK.get());
    }
}