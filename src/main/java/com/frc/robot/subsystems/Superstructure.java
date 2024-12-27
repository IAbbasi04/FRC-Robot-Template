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
    
    private EncoderSim pivotEncoderSim = EncoderSim.createForIndex(0);
    private EncoderSim elevatorEncoderSim = EncoderSim.createForIndex(1);


    private double desiredPivotRadians = 0d;
    private double desiredElevatorMeters = 0d;

    private PIDProfile ELEVATOR_GAINS = new PIDProfile()
        .setP(3d)
        .setD(1E-4)
        .setV(11.08)
        .setA(0.01);

    private PIDProfile PIVOT_GAINS = new PIDProfile()
        .setP(5d)
        .setD(1E-2)
        .setV(5.07)
        .setA(0.02);

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

        this.mech2d = new Mechanism2d(1d, 1d);
        this.mechRoot = mech2d.getRoot("Base", 0.3, 0.0);
        this.mechBase = mechRoot.append(new MechanismLigament2d("Base", 0.2d, 90d));
        this.mechPivot = mechBase.append(new MechanismLigament2d("Pivot", 0.7d, -90d));
        this.mechShooter = mechPivot.append(new MechanismLigament2d("Shooter", 0.4d, 135d));

        // this.elevatorEncoder.setDistancePerPulse(1/4096);
        // this.pivotEncoder.setDistancePerPulse(2*Math.PI/4096);
    }

    public void setPivotAngleRadians(double desiredRadians) {
        this.desiredPivotRadians = desiredRadians;
    }

    public void setElevatorPositionMeters(double desiredMeters) {
        this.desiredElevatorMeters = desiredMeters;
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
        // TODO - StaticArmSim, ExtendingArmSim

        double appliedElevator = this.ELEVATOR_GAINS.toPIDController().calculate(
            elevatorSim.getPositionMeters(), 
            desiredElevatorMeters
        );
        
        double appliedPivot = this.PIVOT_GAINS.toPIDController().calculate(
            pivotSim.getAngleRads() + Math.PI/2d,
            desiredPivotRadians
        );

        this.logger.log("Applied Elevator Voltage", 12d * appliedElevator);
        this.logger.log("Applied Pivot Voltage", 12d * appliedPivot);
            
        elevatorSim.setInput(appliedElevator * RobotController.getBatteryVoltage());
        pivotSim.setInput(appliedPivot * RobotController.getBatteryVoltage());

        elevatorSim.update(Robot.CLOCK.dt());
        pivotSim.update(Robot.CLOCK.dt());

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(pivotSim.getCurrentDrawAmps()));

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));

        pivotEncoderSim.setDistance(pivotSim.getAngleRads());
        elevatorEncoderSim.setDistance(elevatorSim.getPositionMeters());

        mechPivot.setAngle(Math.toDegrees(pivotSim.getAngleRads()));
        mechPivot.setLength(elevatorSim.getPositionMeters());
    }

    @Override
    public void periodicLogs() {
        this.logger.log("Desired Pivot Percent Output", this.desiredPivotRadians);
        this.logger.log("Desired Elevator Percent Output", this.desiredElevatorMeters);
        this.logger.log("Current Pivot Radians", this.pivotSim.getAngleRads() + Math.PI/2d);
        this.logger.log("Current Elevator Meters", this.elevatorSim.getPositionMeters());
    }

    @Override
    public void stop() {
        this.setPivotAngleRadians(0d);
        this.setElevatorPositionMeters(0d);
    }
}