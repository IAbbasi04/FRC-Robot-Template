package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    public void setInches(double inches) {
        this.desiredInches = inches;
        double pid = positionCtrl.toProfiledPIDController().calculate(getCurrentInches(), inches);
        double ff = positionCtrl.feedForward.calculate(elevatorSim.getVelocityMetersPerSecond());
        double voltage = pid + ff;

        elevatorSim.setInputVoltage(voltage);
    }

    @Override
    public double getCurrentInches() {
        return Units.metersToInches(elevatorSim.getPositionMeters() - BASE_STAGE_HEIGHT_METERS);
    }

    @Override
    public void halt() {
        elevatorSim.setInput(0);
    }

    @Override
    public void updateInputs() {
        elevatorSim.update(0.02);
        elevator.setLength(Units.inchesToMeters(getCurrentInches()) + BASE_STAGE_HEIGHT_METERS);
    }
}