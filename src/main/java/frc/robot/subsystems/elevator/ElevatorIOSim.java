package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.*;
import lib.team8592.PIDProfile;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

public class ElevatorIOSim extends ElevatorIO {
    private ElevatorSim elevatorSim =  new ElevatorSim(
        DCMotor.getFalcon500(1), 
        1d / GEAR_RATIO, 
        CARRIAGE_MASS_KG,
        DRUM_RADIUS_METERS,
        BASE_STAGE_HEIGHT_METERS, 
        BASE_STAGE_HEIGHT_METERS + MAX_EXTENSION_METERS,
        false, 
        0d
    );

    private final PIDProfile positionCtrl = new PIDProfile()
        .setP(1d)
        .setV(5d)
        .setA(0.02)
        .setS(0.01);

    private Mechanism2d elevatorMech = new Mechanism2d(1d, 2d);
    private MechanismRoot2d elevatorRoot = elevatorMech.getRoot("Root", 0.5, 0d);
    private MechanismLigament2d elevator = elevatorRoot.append(new MechanismLigament2d("Base Stage", BASE_STAGE_HEIGHT_METERS, 90));

    public ElevatorIOSim() {
        SmartDashboard.putData("Elevator Mechanism2d", elevatorMech);
    }

    @Override
    public void setPosition(double inches) {
        this.desiredPosition = inches;
        double pid = positionCtrl.toProfiledPIDController().calculate(getPosition(), inches);
        double ff = positionCtrl.feedForward.calculate(elevatorSim.getVelocityMetersPerSecond());
        double voltage = pid + ff;

        elevatorSim.setInputVoltage(voltage);
    }

    @Override
    public double getPosition() {
        return Units.metersToInches(elevatorSim.getPositionMeters() - BASE_STAGE_HEIGHT_METERS);
    }

    @Override
    public void updateInputs() {
        elevatorSim.update(0.02);
        elevator.setLength(Units.inchesToMeters(getPosition()) + BASE_STAGE_HEIGHT_METERS);
    }
}