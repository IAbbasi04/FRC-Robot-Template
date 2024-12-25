package com.frc.robot.subsystems;

import com.frc.robot.Robot;
import com.frc.robot.Suppliers;
import com.lib.team8592.MatchMode;
import com.lib.team8592.PIDProfile;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.*;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class Superstructure extends NewtonSubsystem {
    private Mechanism2d mech2d;
    private MechanismRoot2d mechRoot;
    private MechanismLigament2d mechBase;
    private MechanismLigament2d mechPivot;
    @SuppressWarnings("unused") private MechanismLigament2d mechShooter;

    private SingleJointedArmSim pivotSim;
    private ElevatorSim elevatorSim;

    private EncoderSim pivotEncoder = EncoderSim.createForIndex(0);
    private EncoderSim elevatorEncoder = EncoderSim.createForIndex(1);

    private double desiredPivot = 0d;
    private double desiredElevator = 0d;

    private PIDProfile ELEVATOR_GAINS = new PIDProfile()
        .setP(1)
        .setV(0.01)
        .setA(0.01);

    protected Superstructure(boolean logToShuffleboard) {
        super(logToShuffleboard);

        this.pivotSim = new SingleJointedArmSim(
            DCMotor.getNeoVortex(1), 
            300, 
            SingleJointedArmSim.estimateMOI(1d, 10), 
            1d,
            -Math.PI/2d, 
            0,
            false, 
            -Math.PI/2d
        );

        this.elevatorSim = new ElevatorSim(
            ELEVATOR_GAINS.kV, 
            ELEVATOR_GAINS.kA, 
            DCMotor.getNeoVortex(1), 
            0.7d, 
            1d, 
            false, 
            0.7d
        );

        mech2d = new Mechanism2d(1d, 1d);
        mechRoot = mech2d.getRoot("Base", 0.3, 0.0);
        mechBase = mechRoot.append(new MechanismLigament2d("Base", 0.2d, 90d));
        mechPivot = mechBase.append(new MechanismLigament2d("Pivot", 0.7d, -90d));
        mechShooter = mechPivot.append(new MechanismLigament2d("Shooter", 0.4d, 135d));

        elevatorEncoder.setDistancePerPulse(1/4096);
        pivotEncoder.setDistancePerPulse(2*Math.PI/4096);
    }

    public void setPivotAngleRadians(double desiredRadians) {
        this.desiredPivot = desiredRadians;
    }

    public void setElevatorPositionMeters(double desiredMeters) {
        this.desiredElevator = desiredMeters;
    }

    @Override
    public void onRobotInit() {
        if (!Suppliers.robotIsReal.getAsBoolean()) {
            this.logger.addSendable("Superstructure", mech2d);
        }
    }
    
    @Override
    public void onInit(MatchMode mode) {}

    @Override
    public void simulationPeriodic() {
        // elevatorSim.setInput(desiredElevator * RobotController.getBatteryVoltage());
        pivotSim.setInput(desiredPivot * RobotController.getBatteryVoltage());
        
        double appliedElevator = this.ELEVATOR_GAINS.toPIDController().calculate(elevatorEncoder.getDistance(), desiredElevator);
        elevatorSim.setInput(appliedElevator/RobotController.getBatteryVoltage());

        elevatorSim.update(Robot.CLOCK.dt());
        pivotSim.update(Robot.CLOCK.dt());

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(pivotSim.getCurrentDrawAmps()));

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));

        pivotEncoder.setDistance(pivotSim.getAngleRads());
        elevatorEncoder.setDistance(elevatorSim.getPositionMeters());

        mechPivot.setAngle(Math.toDegrees(pivotSim.getAngleRads()));
        mechPivot.setLength(elevatorSim.getPositionMeters());
    }

    @Override
    public void periodicLogs() {
        this.logger.log("Desired Pivot Percent Output", this.desiredPivot);
        this.logger.log("Desired Elevator Percent Output", this.desiredElevator);
        this.logger.log("Pivot Encoder Value", pivotEncoder.getDistance());
        this.logger.log("Elevator Encoder Value", elevatorEncoder.getDistance());
    }

    @Override
    public void stop() {
        this.setPivotAngleRadians(0d);
        this.setElevatorPositionMeters(0d);
    }
}