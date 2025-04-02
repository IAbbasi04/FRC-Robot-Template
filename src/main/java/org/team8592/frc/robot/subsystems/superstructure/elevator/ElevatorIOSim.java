package org.team8592.frc.robot.subsystems.superstructure.elevator;

import org.team8592.frc.robot.Robot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class ElevatorIOSim extends ElevatorIO {
    private ElevatorSim elevatorSim = new ElevatorSim(
        DCMotor.getKrakenX60(2),
        4.0,
        Units.lbsToKilograms(30),
        Units.inchesToMeters(1),
        Units.inchesToMeters(30),
        Units.inchesToMeters(60),
        false,
        Units.inchesToMeters(30)
    );

    private Mechanism2d elevatorMech = new Mechanism2d(2d, 2d);
    private MechanismRoot2d root = elevatorMech.getRoot("Elevator Root", 0.1, 0.1);
    private MechanismLigament2d elevator = root.append(new MechanismLigament2d("Elevator", Units.inchesToMeters(30), 90d));

    public ElevatorIOSim() {
        SmartDashboard.putData("Elevator Sim", elevatorMech);
    }

    @Override
    public void setInches(double inches) {
        elevatorSim.setState(Units.inchesToMeters(inches), 0d);
    }

    @Override
    public double getInches() {
        return Units.metersToInches(elevatorSim.getPositionMeters());
    }

    @Override
    public void setPercentOutput(double percentOutput) {
        elevatorSim.setInput(percentOutput * 12d);
    }

    @Override
    public void updateInputs() {
        elevatorSim.update(Robot.CLOCK.dt());
        this.elevator.setLength(getInches());
    }
}