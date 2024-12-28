package com.frc.robot.subsystems;

import com.frc.robot.Robot;
import com.lib.team8592.MatchMode;
import com.lib.team8592.PIDProfile;
import com.lib.team8592.Utils;
import com.lib.team8592.hardware.motor.spark.SparkFlexMotor;
import com.lib.team8592.simulation.SimUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TEST_Intake extends NewtonSubsystem {
    private SparkFlexMotor rollerMotor = null;
    private DCMotorSim rollerMotorSim = null;
    private TrapezoidProfile motionProfile = null;
    private PIDController rollerCtrl = null;
    
    private Timer timer = new Timer();

    private double desiredRollerRPM = 0d;
    private double lastDesiredRollerRPM = 0d;
    private double appliedRollerRPM = 0d;

    private double rollerGearRatio = 0.5; // 1:2 ratio

    private PIDProfile rollerGains = new PIDProfile()
        .setP(1E-3d)
        .setMaxVelocity(5000d)
        .setMaxAcceleration(5000d);

    public TEST_Intake(boolean logToShuffleboard) {
        super(logToShuffleboard);
        this.rollerMotor = new SparkFlexMotor(29);
        this.rollerMotorSim = SimUtils.createSimSparkFlex(
            rollerGearRatio,
            Utils.getMOIForRoller(
                Units.kilogramsToLbs(2d), 
                Units.inchesToMeters(1d)
            )
        );

        this.rollerMotor.withGains(rollerGains);

        this.motionProfile = new TrapezoidProfile(
            new Constraints(
                rollerGains.maxVelocity, 
                rollerGains.maxAcceleration
            )
        );

        this.rollerCtrl = rollerGains.toPIDController();
    }

    public void setRollerVelocity(double desiredRPM) {
        if (!isEnabled()) return; // Do nothing if subsystem not in active list

        if(lastDesiredRollerRPM != desiredRollerRPM) {
            timer.restart();
        }

        this.lastDesiredRollerRPM = desiredRollerRPM;
        this.desiredRollerRPM = desiredRPM;

        this.rollerMotor.setVelocity(desiredRPM);

        if (Robot.isReal()) return; // Only continue in simulation

        this.rollerMotorSim.setInputVoltage(
            rollerGearRatio * 
            desiredRollerRPM / 
            rollerMotor.getVoltageToRPMRatio()
        );
    }

    public double getDesiredVelocity() {
        return this.desiredRollerRPM;
    }

    public double getCurrentVelocity() {
        if (Robot.isSimulation()) {
            return this.rollerMotorSim.getAngularVelocityRPM();
        }
        return this.rollerMotor.getVelocityRPM();
    }

    public State getCurrentState() {
        return new State(0d, getCurrentVelocity());
    }

    public State getDesiredState() {
        return new State(0d, getDesiredVelocity());
    }

    @Override
    public void onInit(MatchMode mode) {
        this.stop();
    }

    @Override
    public void simulationPeriodic() {}

    @Override
    public void periodicLogs() {
        this.appliedRollerRPM = motionProfile.calculate(
            timer.get(),
            getCurrentState(),
            getDesiredState()
        ).velocity;

        this.logger.log("Current Velocity RPM", getCurrentVelocity());
        this.logger.log("Motion Profile Velocity", appliedRollerRPM);
        this.logger.log("PID Controller Output", 
            rollerCtrl.calculate(
                getCurrentVelocity(), 
                appliedRollerRPM
            )
        );
    }

    @Override
    public void stop() {
        this.setRollerVelocity(0d);
    }
}